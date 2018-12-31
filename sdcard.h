/*
 * sdcard.h
 *
 *  Created on: Apr 15, 2017
 *      Author: kdolgikh
 */

#ifndef _SDCARD_H
#define _SDCARD_H

#include "stdint.h"
#include "msp430f5659.h"

#define TRUE								1
#define FALSE								0


// SPI port definitions
#define SDC_PxSEL         					P8SEL
#define SDC_PxDIR         					P8DIR
#define SDC_PxIN          					P8IN
#define SDC_PxOUT         					P8OUT
#define SDC_SIMO          					BIT5
#define SDC_SOMI          					BIT6
#define SDC_UCLK          					BIT4

// Chip Select
#define SDC_CS_PxDIR      					P8DIR
#define SDC_CS_PxOUT      					P8OUT
#define SDC_CS            					BIT7

// commands related defines
#define SDC_INIT_BYTES						10		// Number of bytes sent to SD-card during initialization
#define SDC_NOP 							0xFF	// Dummy send / CRC don't care
#define SDC_ZERO_ARGUMENT					0		// If a command doesn't require argument, it should be 0
#define SDC_CMD_MSB							0X40	// MSB of any sdc command is 0x40
#define SDC_CMD1_CONST_CRC					0x95	// CMD1 CRC plus the last bit of the command yields 0x95
#define N_CR_MAX							8		// Max command response time Ncr, Byte
#define N_CX_MAX							8		// Max response time to read CSD command, Byte

#define SDC_CMD_LGTH						6		// Length of the command, Byte
#define SDC_RW_BLOCK_SIZE					512		// SD-card block size (read/write unit)
#define SDC_CSD_CID_REG_LENGTH				16		// Length of the CSD and CID registers, Byte


// SD-card responses:

// R1 response to any command except SEND_STATUS, 1 Byte
#define SDC_R1_SUCCESS      				0x00
#define SDC_R1_IDLE_STATE					0x01	// The card is in idle state and running initializing process
#define SDC_R1_ERASE_RESET					0x02	// Erase sequence was cleared before executing because ERASE_SEQ_ERR was received
#define SDC_R1_ILLEGAL_CMD					0x04
#define SDC_R1_COM_CRC_ERR					0x08
#define SDC_R1_ERASE_SEQ_ERR				0x10	// An error in the sequence of erase commands
#define SDC_R1_ADDR_ERROR					0x20	// Misaligned address
#define SDC_R1_PARAM_ERROR					0x40	// Command’s argument (address, block length) was out of the allowed range for this card

// R2 response to SEND_STATUS, 2 Bytes: Byte 1 (MSB) is identical to R1, Byte 2 (LSB) below:
#define SDC_R2_SUCCESS      				0x00
#define SDC_R2_CARD_LOCKED					0x01
#define SDC_R2_WP_ERASE_SKIP				0x02	// Also, Lock/Unlock Failed
#define SDC_R2_ERROR						0x04	// General or unknown error
#define SDC_R2_CC_ERROR						0x08	// Card Controller error
#define SDC_R2_CARD_ECC_FAILED				0x10
#define SDC_R2_WP_VIOLATION					0x20
#define SDC_R2_ERASE_PARAM					0x40	// Invalid sector for erase

// R3 response to READ_OCR, 5 Bytes: Byte 1 - R1, Bytes 2-5 - OCR

// Data write response token, page 104
// xxx00101 & 0x1F
// xxx01011 & 0x1F
// xxx01101 & 0x1F
#define SDC_R_TOKEN_DATA_ACCEPTED			0x05	// data to be written is accepted
#define SDC_R_TOKEN_DATA_RJCT_CRC			0x0B	// data to be written is rejected due to CRC error
#define SDC_R_TOKEN_DATA_RJCT_WRITE_ERR		0x0D	// data to be written is rejected due to write error

// Data read/write tokens, page 104
#define SDC_START_SINGLE_BLOCK_READ		    0xFE   // Data token start byte. Send by SD-card
#define SDC_START_MULTIPLE_BLOCK_READ  		0xFE   // Data token start byte. Send by SD-card
#define SDC_START_SINGLE_BLOCK_WRITE   		0xFE   // Data token start byte. Send by host
#define SDC_START_MULTIPLE_BLOCK_WRITE 		0xFC   // Data token start byte. Send by host
#define SDC_STOP_MULTIPLE_BLOCK_WRITE  		0xFD   // Data token stop byte. Send by host

#define READ_ACCESS_DELAY					1000	// Delay (Bytes) between the last bit of the read cmd and the first bit of data

// Data read token, page 105
#define SDC_DATA_ERR_TOKEN_ERROR			0x01
#define SDC_DATA_ERR_TOKEN_CC_ERROR			0x02
#define SDC_DATA_ERR_TOKEN_CARD_ECC_FAILED	0x04
#define SDC_DATA_ERR_OUT_OF_RANGE			0x08

#define SDC_BUSY							0x00	// SD-card holds SOMI Low while busy

// Status Register (different from SD Status Register)
// Only 16 bits containing errors relevant to SPI can be read out of 32 bits SR


// error/success statuses
#define SDC_SUCCESS           				0x00	// any SD-card operation is successful
//#define SDC_BLOCK_SIZE_ERROR  				0x01	// wrong block size was entered
#define SDC_BLOCK_SET_ERROR   				0x02	// command to set block length returned an error
#define SDC_WCMD_ERROR  	  				0x03	// write (W) command returned an error
#define SDC_WDATA_ERROR						0x04	// write (W) operation failed
#define SDC_RCMD_ERROR		  				0x05	// read (R) command returned an error
#define SDC_RDATA_ERROR		  				0x06	// read operation failed
#define SDC_INIT_ERROR        				0x07	// initialization failed
#define SDC_SEND_STATUS_ERROR				0x08	// Error during Send Status command
/*
#define SDC_CRC_ERROR         				0x10
#define SDC_TIMEOUT_ERROR     				0x11
*/


// CMD-number mnemonic in HEX, see pages 99-102 SanDisk SD Card
#define SDC_GO_IDLE_STATE         		 	0x00    //CMD0
#define SDC_SEND_OP_COND          		 	0x01    //CMD1
#define SDC_READ_CSD             		  	0x09    //CMD9
#define SDC_READ_CID             		  	0x0A    //CMD10
#define SDC_STOP_TRANSMISSION    		  	0x0C    //CMD12
#define SDC_SEND_STATUS          		  	0x0D    //CMD13
#define SDC_SET_BLOCKLEN         		  	0x10    //CMD16
#define SDC_READ_SINGLE_BLOCK    		  	0x11    //CMD17
#define SDC_READ_MULTIPLE_BLOCK  		  	0x12    //CMD18
#define SDC_WRITE_BLOCK          		  	0x18    //CMD24
#define SDC_WRITE_MULTIPLE_BLOCK 		  	0x19    //CMD25
#define SDC_WRITE_CSD            		  	0x1B    //CMD27
#define SDC_SET_WRITE_PROT       		  	0x1C    //CMD28
#define SDC_CLR_WRITE_PROT       		  	0x1D    //CMD29
#define SDC_SEND_WRITE_PROT      		  	0x1E    //CMD30
#define SDC_ERASE_WR_BLK_START_ADDR			0x20    //CMD32
#define SDC_ERASE_WR_BLK_END_ADDR   		0x21    //CMD33
#define SDC_EREASE                 			0x26    //CMD38
#define SDC_LOCK_UNLOCK						0x2A	//CMD42
#define	SDC_APP_CMD				   			0x37	//CMD55
#define SDC_READ_OCR						0x3A	//CMD58
#define SDC_CRC_ON_OFF						0x3B	//CMD59


#define SDC_128MB   128                             // SD standard capacity card sizes
#define SDC_256MB   256
#define SDC_512MB   512
#define SDC_1024MB  1024
#define SDC_2048MB  2048


extern uint16_t sdc_read_access_time;
extern uint8_t sdcInitialized_event;
//extern uint8_t sdc_buffer[SDC_RW_BLOCK_SIZE];
extern uint8_t sdc_register_buff[SDC_CSD_CID_REG_LENGTH];
extern uint8_t sdc_cmd_frame[SDC_CMD_LGTH];
extern uint32_t sdc_block_address;                 	// Address of the block on SD-card in bytes
extern uint16_t sdcSize;                            // Size of SD-card. User enters during setup

extern uint8_t sdc_response;						// Response from SD-card returned by SD-card functions

extern uint8_t r1_response;							// R1 response or the first byte of R2 response
extern uint8_t r2_response;							// The second byte of R2 response
extern uint8_t data_resp_token;
extern uint8_t data_response;
extern uint8_t busy_response;

uint32_t num_segments_sdc (uint16_t sdCardSize);


// initialize CS pin for SD-card
void sdc_CS_Init(void);

// Initialize SPI for SD-card
void sdc_SPI_Init(void);

// When SD-card is powered off, pull CS and data lines HIGH,
// pull clock LOW to prevent floating inputs
void sdcDeactivatePorts(void);


void sdcPowerOn (void);


void sdcPowerOff (void);


// send command to SDC
void sdcSendCmd (const uint8_t cmd, uint32_t data, const uint8_t crc);

void sdcGet_R1_R2_Response(uint8_t r2RespRequired);

// Put SD-card into SPI mode
uint8_t sdcStart (void);

// set SDC block length of count=2^n Byte
void sdcSetBlockLength (const uint16_t blocklength);

void sdcCheckBusy(void);

void sdcStopTransmission (void);

// read a size Byte big block beginning at the address.
uint8_t sdcReadSingleBlock(const uint32_t address, const uint16_t count, uint8_t *pBuffer);

// write a 512 Byte big block beginning at the address
uint8_t sdcWriteSingleBlock (const uint32_t address, uint8_t *pBuffer);

uint8_t sdcWriteMultipleBlocks (const uint32_t address, uint8_t *pBuffer, uint32_t count);

// Read Register arg1 with Length arg2 (into the buffer)
uint8_t sdcReadRegister (const uint8_t cmd_register, uint8_t *pBuffer);

#endif /* _SDCARD_H */
