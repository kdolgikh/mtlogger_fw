/*
 * sdcard.c
 *
 *  Created on: Apr 15, 2017
 *      Author: kdolgikh
 */

#ifndef _SDCARD_C
#define _SDCARD_C
//
//---------------------------------------------------------------

#include "hal_SPI.h"
#include "sdcard.h"
#include "sleep_timer.h"

uint8_t sdcInitialized_event = FALSE;
//uint8_t sdc_buffer[SDC_RW_BLOCK_SIZE] = {0};
uint8_t sdc_register_buff[SDC_CSD_CID_REG_LENGTH] = {0};
uint8_t sdc_cmd_frame[SDC_CMD_LGTH] = {0};
uint32_t sdc_block_address = 0; // address is in bytes

// Calculate this value for each card separately
// by using values from CSD register, see page 35, 109
uint16_t sdc_read_access_time = READ_ACCESS_DELAY;

uint16_t sdcSize = SDC_128MB;

uint8_t sdc_response = 0;

uint8_t r1_response = 0;		// R1 response or the first byte of R2 response
uint8_t r2_response = 0;		// The second byte of R2 response
uint8_t data_resp_token = 0;
uint8_t data_response = 0;
uint8_t busy_response = 0;

uint16_t nac = 0;

uint32_t num_segments_sdc (uint16_t sdCardSize)
{
    uint32_t num_segments;  // number of 512 byte segments on the SD card of specific size if using raw write (without File System)

    switch(sdCardSize)
    {
        case(SDC_128MB):
            num_segments = 248320; //3; //262144; //248320 from WinHex
        	break;
        case(SDC_256MB):
            num_segments = 524288; // need to check
        	break;
        case(SDC_512MB):
            num_segments = 1048576; // need to check
        	break;
        case(SDC_1024MB):
            num_segments = 2097152; // need to check
        	break;
        case(SDC_2048MB):
            num_segments = 4194304; // need to check
        	break;
    }

    return num_segments;
}


void sdc_CS_Init(void)
{
    // Configure Chip Select
    SDC_CS_PxOUT &= ~SDC_CS;         // CS LOW -> OK after I unmounted R11
                                    // PxOUT is undefined after reset, hence configure before PxDIR
    SDC_CS_PxDIR |= SDC_CS;         // Set CS DIR to output
}

void sdc_SPI_Init(void)
{
    UCB1CTL1 |= UCSWRST;            // Hold USCI in reset

    if (!sdcInitialized_event)
        {
        UCB1CTL0 |= UCMST;          // 3-pin, 8-bit master mode
        UCB1CTL0 |= UCCKPL;         // Clock polarity
        UCB1CTL0 |= UCMSB;          // MSB first
        UCB1CTL0 |= UCSYNC;         // SPI mode
        UCB1CTL1 |= UCSSEL__SMCLK;  // BRCLK = SMCLK (20MHz)
        UCB1BR0 = 0x40;             // Divide BRCLK by 64 to get SCLK of 312.5 KHz
                                    // During sd-card init SCLK should be > 400KHz
        UCB1BR1 = 0;                // Set to 0 to use only UCB1BR0 value

        SDC_PxSEL |= SDC_SIMO + SDC_SOMI + SDC_UCLK;    // Activate USCI functionality on port 8
        }
    else
        UCB1BR0 = 0x00;             // 0 for SCLK of 20 MHz, 10 for SCLK of 2 MHz

    UCB1CTL1 &= ~UCSWRST;           // Start USCI state machine
}

void sdcDeactivatePorts(void)
{
    UCB1CTL1 |= UCSWRST;                            // Hold USCI in reset
    SDC_PxSEL &= ~(SDC_SIMO + SDC_SOMI + SDC_UCLK); // Deactivate USCI functionality on port 8
    SDC_PxOUT &= ~(SDC_SIMO + SDC_SOMI + SDC_UCLK); // Set LOW
    SDC_PxDIR |= SDC_SIMO + SDC_SOMI + SDC_UCLK;    // Set output direction
}

void sdcPowerOn (void)
{
    SDC_CS_PxOUT |= SDC_CS;         // Set CS HIGH
    P4OUT |= BIT7;                  // turn SD_VCC on
    sdc_SPI_Init();
    Sleep_Timer_Fast(40);           // Wait 10 ms until sd-card initializes
}

void sdcPowerOff (void)
{
    sdcDeactivatePorts();
    P4OUT &= ~BIT7;                 // turn SD_VCC off
    SDC_CS_PxOUT &= ~SDC_CS;        // Set CS LOW
    sdcInitialized_event = FALSE;   // SD-card will wake up in uninitialized state after power up
}


// send command to SD
void sdcSendCmd (const uint8_t cmd, uint32_t data, const uint8_t crc)
{
	sdc_cmd_frame[0]=(cmd | SDC_CMD_MSB);
	sdc_cmd_frame[1] = (data >> 24) & 0xFF;		//MSB
	sdc_cmd_frame[2] = (data >> 16) & 0xFF;
	sdc_cmd_frame[3] = (data >> 8) & 0xFF;
	sdc_cmd_frame[4] = data & 0xFF;				//LSB
	sdc_cmd_frame[5] = crc;

	spiSendFrame(sdc_cmd_frame,6);
}


// sdc Get R1 or R2 Response (when sending CMD13)
void sdcGet_R1_R2_Response(uint8_t r2ResponseRequired)
{
	//Response comes 1-8 bytes after command
	//data will be 0xFF until response

	uint8_t i = 0;
	uint8_t break_flag = FALSE;

	while((i <= N_CR_MAX) && (!break_flag)) // Ncr is 1 to 8 bytes, byte 9 - response, hence <= sign
	{
		r1_response = spiSendByte(SDC_NOP);

		if (r1_response != SDC_NOP)			// It is the task of overlaying function to check that response is valid
			break_flag = TRUE;

/*		switch (response)
		{
		case (SDC_R1_SUCCESS):
		case (SDC_R1_IDLE_STATE):
		case (SDC_R1_ERASE_RESET):
		case (SDC_R1_ILLEGAL_CMD):
		case (SDC_R1_COM_CRC_ERR):
		case (SDC_R1_ERASE_SEQ_ERR):
		case (SDC_R1_ADDR_ERROR):
		case (SDC_R1_PARAM_ERROR):
			break_flag = TRUE;
		}*/

		i++;
	}

	if (r2ResponseRequired)
		r2_response = spiSendByte(SDC_NOP);

	spiSendByte(SDC_NOP);					// 8 clock cycles are required after the response
											// See page 53 SD physical layer spec. v2.00, p 96 SanDisk UG
}


// Initialize SD card in SPI mode
uint8_t sdcStart (void)
{
	uint8_t i;

	//initialization sequence on PowerUp

	// Send > 74 SCLK ticks (in this case 8 bits/byte * 10 bytes = 80 bits)
	for(i = 0; i < SDC_INIT_BYTES ; i++)
		spiSendByte(SDC_NOP);

	// Set CS Low
	SDC_CS_PxOUT &= ~SDC_CS;

	//Send Command 0 to put SD in SPI mode
	sdcSendCmd(SDC_GO_IDLE_STATE, SDC_ZERO_ARGUMENT, SDC_CMD1_CONST_CRC);

	//Response should be 0x01
	sdcGet_R1_R2_Response(FALSE);

	// CS High
	SDC_CS_PxOUT |= SDC_CS;

	if(r1_response != SDC_R1_IDLE_STATE)
		return SDC_INIT_ERROR;

	//Wait until card finishes initializing and sends 0x00 (as the only possible response for this command)
	while(r1_response == SDC_R1_IDLE_STATE)
	{
		// Increase delay, if necessary
		spiSendByte(SDC_NOP);		// 8 clock cycles are used as delay between CS transitions
									// SD-card starts counting clocks after CS is asserted Low

		SDC_CS_PxOUT &= ~SDC_CS; 	// CS Low

		sdcSendCmd(SDC_SEND_OP_COND, SDC_ZERO_ARGUMENT, SDC_NOP); 	// External delay is not required here, because
																	// function prepares cmd for some time
		sdcGet_R1_R2_Response(FALSE);

		SDC_CS_PxOUT |= SDC_CS;
	}

	sdcInitialized_event = TRUE;	// This flag is used to increase SCLK after initialization

	return SDC_SUCCESS;
}


// Set block_length
// 1 < blocklength < 512
void sdcSetBlockLength (const uint16_t blocklength)
{
	SDC_CS_PxOUT &= ~SDC_CS;

	spiSendByte(SDC_NOP); // Ncs

	// Set the block length
	sdcSendCmd(SDC_SET_BLOCKLEN, blocklength, SDC_NOP);

	// Receive R1 response
	sdcGet_R1_R2_Response(FALSE);

	SDC_CS_PxOUT |= SDC_CS;
}


// Check if SD-card is busy
void sdcCheckBusy(void)
{
	// Wait until SD-card is not busy
	do
	{
		busy_response = spiSendByte(SDC_NOP);
	}
	while(busy_response == SDC_BUSY); 	// SD-card holds SOMI Low when busy

	spiSendByte(SDC_NOP);	//Nwr (page 109 SanDisk) or Nec (page 108 SanDisk)
}


uint8_t sdcGetDataResponse(void)
{
	data_resp_token = spiSendByte(SDC_NOP);
	data_resp_token &= 0x1F;
	return data_resp_token;
}

void sdcStopTransmission (void)
{
		spiSendByte(SDC_NOP); 	// Ncr, see page 108 SanDisk "Card Response to Host Command"
								// Additional time between two commands

		// Send Stop Transmission command
		sdcSendCmd(SDC_STOP_TRANSMISSION, SDC_ZERO_ARGUMENT, SDC_NOP);

		// Receive R1 response
		sdcGet_R1_R2_Response(FALSE);
}


// read a size "block_size" block beginning at the "address"
// 1 < block_size < 512
// 1 < buffer_size < 512
uint8_t sdcReadSingleBlock(const uint32_t address, const uint16_t block_size, uint8_t *pBuffer)
{
	// Ideally, length of the buffer is needed to be checked

	uint16_t i = 0;

/*	// Check that block length is less than segment size and larger than 1 byte, see page 12
	if (block_size < 1 || block_size > SDC_RW_BLOCK_SIZE)
		return SDC_BLOCK_SIZE_ERROR;*/

	// Set the block length
	sdcSetBlockLength(block_size);
	if (r1_response == SDC_R1_SUCCESS)   // block length can be set
	{
		SDC_CS_PxOUT &= ~SDC_CS;

		spiSendByte(SDC_NOP); // Ncs

		sdcSendCmd(SDC_READ_SINGLE_BLOCK,address, SDC_NOP); // send read command SDC_READ_SINGLE_BLOCK=CMD17
		sdcGet_R1_R2_Response(FALSE);						// receive R1 response

		if (r1_response == SDC_R1_SUCCESS)
		{
			// Receive data start token
			// Data token for read command will be sent by SD-card after Nac
			// Nac_max = 100*(TAAC*fclk + NSAC*100), see page 109, page 35
			// Where TAAC and NSAC are values from CSD register
			while(i < sdc_read_access_time)
			{
				data_resp_token = spiSendByte(SDC_NOP);
				if(data_resp_token == SDC_START_SINGLE_BLOCK_READ)
					break;
				i++;
				nac++;
			}

			//If desired response was not received during sdc_access_time, return SDC_RDATA_ERROR
			if (data_resp_token != SDC_START_SINGLE_BLOCK_READ)
			{
				SDC_CS_PxOUT |= SDC_CS;
				return SDC_RDATA_ERROR;
			}

			// receive data
			spiReadFrame(pBuffer, block_size);

			// get 2 CRC bytes
			spiSendByte(SDC_NOP);
			spiSendByte(SDC_NOP);

			// required 8 clock cycles after data block
			spiSendByte(SDC_NOP);

			SDC_CS_PxOUT |= SDC_CS;

			return SDC_SUCCESS;
		}
		else
		{
			SDC_CS_PxOUT |= SDC_CS;
			return SDC_RCMD_ERROR;
		}
	}
	else
		return SDC_BLOCK_SET_ERROR;
}


uint8_t sdcWriteSingleBlock (const uint32_t address, uint8_t *pBuffer)
{
	// Set the block length to write, block size should be equal 512 bytes
	sdcSetBlockLength(SDC_RW_BLOCK_SIZE);

	if (r1_response == SDC_R1_SUCCESS)   // block length could be set
	{
		SDC_CS_PxOUT &= ~SDC_CS;

		spiSendByte(SDC_NOP); // Ncs

		// send write command
		sdcSendCmd(SDC_WRITE_BLOCK, address, SDC_NOP);

		sdcGet_R1_R2_Response(FALSE);

		if (r1_response == SDC_R1_SUCCESS)
		{
			// Send the start token
			spiSendByte(SDC_START_SINGLE_BLOCK_WRITE);

			// Send data
			spiSendFrame(pBuffer, SDC_RW_BLOCK_SIZE);

			// Send 2 CRC bytes
			spiSendByte(SDC_NOP);
			spiSendByte(SDC_NOP);

			// Get data response token
			data_response = sdcGetDataResponse();

			if (data_response == SDC_R_TOKEN_DATA_ACCEPTED)
				sdcCheckBusy();	// Check busy

			// Check write status, see page 94 SanDisk
			spiSendByte(SDC_NOP); // Ncr of 1 byte between two commands, see page 108 SanDisk

			sdcSendCmd(SDC_SEND_STATUS, SDC_ZERO_ARGUMENT, SDC_NOP);	// Send Status command

			sdcGet_R1_R2_Response(TRUE);								// Get r1 and r2 responses

			SDC_CS_PxOUT |= SDC_CS;

			if (r1_response == SDC_R1_SUCCESS)
			{
				if (r2_response == SDC_R2_SUCCESS)
					return SDC_SUCCESS;
				else
					return SDC_WDATA_ERROR;
			}
			else
				return SDC_SEND_STATUS_ERROR;
		}
		else
		{
			SDC_CS_PxOUT |= SDC_CS;
			return SDC_WCMD_ERROR;
		}
	}
	else
		return SDC_BLOCK_SET_ERROR;
}


uint8_t sdcWriteMultipleBlocks (const uint32_t address, uint8_t *pBuffer, uint32_t count)
{
	uint32_t i;

	// Set the block length to write, block size should be equal 512 bytes
	sdcSetBlockLength(SDC_RW_BLOCK_SIZE);

	if (r1_response == SDC_R1_SUCCESS)   // block length can be set
	{
		SDC_CS_PxOUT &= ~SDC_CS;

		spiSendByte(SDC_NOP); // Ncs

		// send write command
		sdcSendCmd(SDC_WRITE_MULTIPLE_BLOCK, address, SDC_NOP);

		sdcGet_R1_R2_Response(FALSE);

		if (r1_response == SDC_R1_SUCCESS)
		{
			for (i = 0; i < count; i++)
			{
				// Send the start token
				spiSendByte(SDC_START_MULTIPLE_BLOCK_WRITE);

				// Send data
				spiSendFrame(pBuffer, SDC_RW_BLOCK_SIZE);

				pBuffer +=SDC_RW_BLOCK_SIZE;

				// Send 2 CRC bytes
				spiSendByte(SDC_NOP);
				spiSendByte(SDC_NOP);

				// Get data response token
				data_response = sdcGetDataResponse();

				if (data_response == SDC_R_TOKEN_DATA_ACCEPTED)
					sdcCheckBusy();	// Check busy // Ideally, after busy should check results of the programming (CMD13), see page 94 SanDisk
				else
				{
					sdcStopTransmission(); // stop transmission if error is encountered
					// In case of write error, additional actions are to send CMD13 and ACMD22 (see page 104 SanDisk)

					if (data_response == SDC_R_TOKEN_DATA_RJCT_WRITE_ERR)
					{
						sdcSendCmd(SDC_SEND_STATUS, SDC_ZERO_ARGUMENT, SDC_NOP);	// Send Status command
						sdcGet_R1_R2_Response(TRUE);								// Get r1 and r2 responses

						if (r1_response != SDC_R1_SUCCESS)
						{
							SDC_CS_PxOUT |= SDC_CS;
							return SDC_SEND_STATUS_ERROR;
						}

						// It's the task for overlaying function to check R2 response
						// Checking the num of well written blocks (ACMD22) is not implemented
					}

					SDC_CS_PxOUT |= SDC_CS;

					return data_response;
				}
			}

			// Send the stop token once all data is sent
			spiSendByte(SDC_STOP_MULTIPLE_BLOCK_WRITE);
			spiSendByte(SDC_NOP); 	// Nbr, see page 109 SanDisk
			sdcCheckBusy();			// After Nbr card may send Busy

			SDC_CS_PxOUT |= SDC_CS;

			return SDC_SUCCESS;
		}
		else
		{
			SDC_CS_PxOUT |= SDC_CS;
			return SDC_WCMD_ERROR;
		}
	}
	else
		return SDC_BLOCK_SET_ERROR;
}


// Reading the contents of the CSD and CID registers in SPI mode is a simple read-block transaction.
// CID register length is 16 byte (128 bit), page 32
// CSD register length is 16 byte (128 bit), page 33
// When issuing CSD read, read access time is unknown, use Ncr instead, see page 95
// Issue CID read after CSD read, so read access time is known
uint8_t sdcReadRegister (const uint8_t cmd_register, uint8_t *pBuffer)
{
	// Ideally, length of the buffer is needed to be checked

	uint16_t i = 0;

	sdcSetBlockLength(SDC_CSD_CID_REG_LENGTH);

	if (r1_response == SDC_R1_SUCCESS)
	{
		SDC_CS_PxOUT &= ~SDC_CS;

		spiSendByte(SDC_NOP); // Ncs

		sdcSendCmd(cmd_register, SDC_ZERO_ARGUMENT, SDC_NOP);

		sdcGet_R1_R2_Response(FALSE);

		if (r1_response == SDC_R1_SUCCESS)
		{
			// Receive data start token
			// Data token for read CSD/CID command will be sent by SD-card after Ncx
			// Ncx_max = 8 Bytes, see page 109
			// However, the value of read access time will be used
			while(i < sdc_read_access_time)
			{
				data_resp_token = spiSendByte(SDC_NOP);
				if(data_resp_token == SDC_START_SINGLE_BLOCK_READ)
					break;
				i++;
			}

			//If desired response was not received during sdc_access_time, return SDC_RDATA_ERROR
			if (data_resp_token != SDC_START_SINGLE_BLOCK_READ)
				return SDC_RDATA_ERROR;

			// Receive data
			spiReadFrame(pBuffer, SDC_CSD_CID_REG_LENGTH);

			// Get 2 CRC bytes
			spiSendByte(SDC_NOP);
			spiSendByte(SDC_NOP);

			// Required 8 clock cycles after data block
			spiSendByte(SDC_NOP);

			SDC_CS_PxOUT |= SDC_CS;

			return SDC_SUCCESS;
		}
		else
		{
			SDC_CS_PxOUT |= SDC_CS;
			return SDC_RCMD_ERROR;
		}
	}
	else
		return SDC_BLOCK_SET_ERROR;
}

//---------------------------------------------------------------------
#endif /* _SDCARD_C */
