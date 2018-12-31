/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*
 * send_over_usb.c
 *
 *  Created on: Mar 31, 2017
 *      Author: kdolgikh
 *      Uses TI USB library
 */

#include "stdint.h"
#include "driverlib.h"
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                 				// USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"
#include "send_over_usb.h"
#include "data_prep.h"
#include "crc8.h"
#include "states.h"
#include "adc_driver.h"
#include "sleep_timer.h"
#include "measure.h"
#include "sdcard.h"
#include "real_time_clock.h"
#include "ucs_functions.h"

extern device_state dev_state;
extern uint8_t dev_state_old;

extern volatile uint8_t DRDY_event;

volatile uint8_t USB_DataReceived_event = FALSE;					// Flag showing the reception of the message from PC

uint8_t USB_AbortOperation_event = FALSE;

uint8_t	usb_msg_trx[USB_MSG_TRX_LGTH] = {0};					// Array for the message to/from PC

uint8_t NumDataBytes_Sent = FALSE;

// for debug, move inside functions once done debugging
uint32_t total_bytes = 0;
uint32_t flash_bytes = 0;
uint32_t sdc_address = 0;
uint8_t *flash_address = (uint8_t *)BANK1_START;
uint8_t sdc_fail_count = 0;

extern uint16_t nac; // for debug only

uint8_t ram_stg_temp[SEGMENT_SIZE] = {0}; // for data from RAM, the size will be "index"
uint8_t ram_stg_temp2[SEGMENT_SIZE] = {0}; // for data from sd-card

uint8_t USBconnected = FALSE;

void usb_connect2host (void)
{
	if (USB_enable() == USB_SUCCEED)			// Connect to host
	{
		USB_reset();
		USB_connect();
	}

	LPM0; // Device should be in AM or LPM0 during enumeration.
		  // Wait here until device is enumerated.
}

void usb_command_line (void)
{
    USBCDC_receiveDataInBuffer(usb_msg_trx,USB_MSG_TRX_LGTH,CDC0_INTFNUM);

	switch(*usb_msg_trx)				// the first byte of the message is the command code
	{
		case(GET_CONV_RESULTS):
			usb_send_conv_result();
			break;
		case(USB_GET_NUM_CONV):			// num_conv is uint8_t,  will be changed to uint16_t
		case(USB_GET_ADC_DATA_RATE):
		case(USB_GET_ADC_VREFCON):
		case(USB_GET_DEV_STATE):
		case(USB_GET_SYSOCAL_EN):
		case(USB_GET_DELAY_EN):
		case(USB_GET_KEEP_DATA):
		case(USB_GET_CAL_EN):
			usb_get_dev_param();
			break;
		case(USB_GET_MEAS_RATE):
			usb_get_meas_rate();
			break;
		case(USB_SET_NUM_CONV):
		case(USB_SET_ADC_DATA_RATE):
		case(USB_SET_DEV_STATE):
		case(USB_SET_SYSOCAL_EN):
		case(USB_SET_DELAY_EN):
		case(USB_SET_MEAS_RATE):
		case(USB_SET_MEAS_START):
		case(USB_SET_KEEP_DATA):
		case(USB_SET_CAL_EN):
			usb_set_dev_param();
			break;
		case(USB_SET_ADC_VREFCON):
			usb_set_adc_vrefcon();
			break;
		case(USB_GET_ADC_OFC):
		case(USB_GET_ADC_FSC):
			usb_get_adc_cal_register();
			break;
		case(USB_GET_RTC):
			usb_get_rtc();
			break;
		case(USB_SET_RTC):
			usb_set_rtc();
			break;
		default:
			break;
	}
}


void usb_ack(void)
{
	uint16_t num_tx_bytes = 0;	// number of transmitted bytes over USB, returned by abortSend

	// acknowledge success of the set command by sending the received cmd
	if(USBCDC_sendDataAndWaitTillDone(usb_msg_trx,
									ACK_BUFF_LGTH,
									CDC0_INTFNUM,
									USB_TX_RETRY_NUM))
		USBCDC_abortSend(&num_tx_bytes, CDC0_INTFNUM);
}


void usb_send_conv_result (void)
{
	uint16_t num_tx_bytes = 0;
	uint16_t i;
//	uint8_t sdc_fail_count = 0;
//	uint8_t ram_stg_temp[SEGMENT_SIZE] = {0}; // for data from RAM
//uint8_t ram_stg_temp2[SEGMENT_SIZE] = {0}; // for data from sd-card

/*	uint32_t total_bytes = 0;	// Total num of data bytes
	uint32_t flash_bytes = 0; // Num of data bytes in flash*/

	if (!NumDataBytes_Sent)
	{
		// Let the host know on the number of bytes to receive
		flash_bytes = (uint32_t)(flash_data8_ptr - BANK1_START); // need to make sure that flash_data8_ptr will hold what I need!!!

		if (index > TS_W_CRC_NUM_BYTES)	// Data was written to ram_stg
			total_bytes = sdc_block_address + flash_bytes + (uint32_t)index;
		else							// Data was not written to ram_stg
			total_bytes = sdc_block_address + flash_bytes;

		long_int_divide(&total_bytes,(usb_msg_trx+PARAM_POS),1); 	// 1 is number of uint32_t containing in total_bytes

		if(USBCDC_sendDataAndWaitTillDone(usb_msg_trx,				// send ACK plus num_bytes
										NUM_BYTE_BUFF_LGTH,
										CDC0_INTFNUM,
										USB_TX_RETRY_NUM))
			USBCDC_abortSend(&num_tx_bytes, CDC0_INTFNUM);

		NumDataBytes_Sent = TRUE;
	}
	else
	{
		NumDataBytes_Sent = FALSE;

		// Send ACK to the host // It is easier to send ACK like this than embedding it with data
		usb_ack();

		if(USBconnected)
		{
			if (index > TS_W_CRC_NUM_BYTES)	// Move data from ram_stg into ram_stg_temp since it will be sent the last
			{
				for (i = 0; i < index; i++)
				{
					*(ram_stg_temp+i) = *(ram_stg+i);
				}
			}

			if (sdc_block_address && USBconnected) // Data has been written to SD-card
			{
				sdcPowerOn();

				if (sdcStart() == SDC_SUCCESS)                           // Set SD-card in SPI-mode
					sdc_SPI_Init();

				while ((sdc_address < sdc_block_address) && USBconnected)
				{
					do	// Sometimes SD-card returns 0x06 and 0x02 errors, power cycling helps mitigating this
					{
						sdc_response = sdcReadSingleBlock(sdc_address, SDC_RW_BLOCK_SIZE, ram_stg_temp2);
						if(sdc_response)
						{
							sdcPowerOff();
							sdc_fail_count++;
							sdcPowerOn();
							if (sdcStart() == SDC_SUCCESS)                           // Set SD-card in SPI-mode
								sdc_SPI_Init();
						}
					}
					while (sdc_response && (sdc_fail_count < 4)); // Power cycle 3 times only

					nac = 0; // for debug only
					sdc_fail_count = 0;

					sdc_address += SDC_RW_BLOCK_SIZE;

					reverse_order(ram_stg_temp2,ram_stg,SEGMENT_SIZE); //Reverse order is used because data in SD-card is moved from flash where it is stored in uint32

					check_data_crc(ram_stg);

					if(USBCDC_sendDataAndWaitTillDone(ram_stg,
													SEGMENT_SIZE,
													CDC0_INTFNUM,
													USB_TX_RETRY_NUM))
						USBCDC_abortSend(&num_tx_bytes, CDC0_INTFNUM);
				}
				sdc_address = 0;	// Reinit sdc address

				sdcPowerOff();
			}

			if (flash_bytes && USBconnected)	// Data has been written to flash
			{
				while ((flash_address < flash_data8_ptr) && USBconnected)
				{
					reverse_order(flash_address,ram_stg,SEGMENT_SIZE); //copy data from flash into ram_stg. Reverse order is used because data in flash is uint32

					check_data_crc(ram_stg);

					if(USBCDC_sendDataAndWaitTillDone(ram_stg,
													SEGMENT_SIZE,
													CDC0_INTFNUM,
													USB_TX_RETRY_NUM))
						USBCDC_abortSend(&num_tx_bytes, CDC0_INTFNUM);

					flash_address += SEGMENT_SIZE;
				}
				flash_address = (uint8_t *)BANK1_START;	// Reinit flash address
			}

			if ((index > TS_W_CRC_NUM_BYTES) && USBconnected)
			{
				replace_ram_stg_crc(ram_stg_temp, index);	// replace CRC bytes with "CD"/"BD" values

				if(USBCDC_sendDataAndWaitTillDone(ram_stg_temp,
												index,
												CDC0_INTFNUM,
												USB_TX_RETRY_NUM))
					USBCDC_abortSend(&num_tx_bytes, CDC0_INTFNUM);
			}
		}
	}
}


void usb_get_dev_param (void)
{
	uint16_t num_tx_bytes = 0;	// number of transmitted bytes over USB, returned by abortSend

	switch(*usb_msg_trx)
	{
	case(USB_GET_NUM_CONV):
		// *usb_msg_trx = num_conv;		// when this changes to uint16_t, redesign the function
		break;
	case(USB_GET_ADC_DATA_RATE):
		*(usb_msg_trx + PARAM_POS) = adc_data_rate;
		break;
	case(USB_GET_ADC_VREFCON):
		*(usb_msg_trx + PARAM_POS) = adc_mux1_vrefcon;
		break;
	case(USB_GET_DEV_STATE):
		*(usb_msg_trx + PARAM_POS) = dev_state_old;
		break;
	case(USB_GET_SYSOCAL_EN):
		*(usb_msg_trx + PARAM_POS) = sysocal_enable;
		break;
	case(USB_GET_DELAY_EN):
		*(usb_msg_trx + PARAM_POS) = delay_enable;
		break;
	case(USB_GET_KEEP_DATA):
		*(usb_msg_trx + PARAM_POS) = KeepData;
		break;
	case(USB_GET_CAL_EN):
		*(usb_msg_trx + PARAM_POS) = CalEn;
		break;
	}

	if(USBCDC_sendDataAndWaitTillDone(usb_msg_trx,
									PARAM_BUFF_LGTH,
									CDC0_INTFNUM,
									USB_TX_RETRY_NUM))
		USBCDC_abortSend(&num_tx_bytes, CDC0_INTFNUM);
}


void usb_get_meas_rate(void)
{
	uint16_t num_tx_bytes = 0;	// number of transmitted bytes over USB, returned by abortSend

	long_int_divide(&meas_rate,(usb_msg_trx+PARAM_POS),1); // 1 is the number of uint32 values to convert - only meas_rate

	if(USBCDC_sendDataAndWaitTillDone(usb_msg_trx,
									MEAS_RATE_BUFF_LGTH,
									CDC0_INTFNUM,
									USB_TX_RETRY_NUM))
		USBCDC_abortSend(&num_tx_bytes, CDC0_INTFNUM);
}


void usb_get_adc_cal_register(void)
{
	uint16_t num_tx_bytes = 0;

	uint8_t *cal_reg_cmd_ptr; 	// pointer to the command that reads a calibration register

	if (*usb_msg_trx == USB_GET_ADC_OFC)
		cal_reg_cmd_ptr = adc_rreg_ofc;
	else if (*usb_msg_trx == USB_GET_ADC_FSC)
		cal_reg_cmd_ptr = adc_rreg_fsc;

	adcWorkInProgress = TRUE;

	P9OUT |= ADC_START;

	adc_spi_rreg_cal_register(cal_reg_cmd_ptr, (usb_msg_trx + PARAM_POS));

	P9OUT &= ~ADC_START;

	LPM0;

	adcWorkInProgress = FALSE;

	if(USBCDC_sendDataAndWaitTillDone(usb_msg_trx,
								CAL_REG_BUFF_LGTH,
								CDC0_INTFNUM,
								USB_TX_RETRY_NUM))
		USBCDC_abortSend(&num_tx_bytes, CDC0_INTFNUM);
}


void usb_get_rtc(void)
{
	uint16_t num_tx_bytes = 0;

	RTCCTL0 &= ~RTCRDYIFG;	// Clear RTCRDY IFG
	RTCCTL0 |= RTCRDYIE;	// Enable RTCRDY interrupt
	LPM0;	// Sleep until RTCRDY is HIGH

	*(usb_msg_trx + RTC_YEAR_POS)=(uint8_t)RTCYEAR;
	*(usb_msg_trx + RTC_MON_POS)=RTCMON;
	*(usb_msg_trx + RTC_DAY_POS)=RTCDAY;
	*(usb_msg_trx + RTC_HOUR_POS)=RTCHOUR;
	*(usb_msg_trx + RTC_MIN_POS)=RTCMIN;
	*(usb_msg_trx + RTC_SEC_POS)=RTCSEC;

	if(USBCDC_sendDataAndWaitTillDone(usb_msg_trx,
									USB_RTC_BUFF_LGTH,
									CDC0_INTFNUM,
									USB_TX_RETRY_NUM))
		USBCDC_abortSend(&num_tx_bytes, CDC0_INTFNUM);
}


void usb_set_dev_param(void)
{
	switch(*usb_msg_trx)
	{
	case(USB_SET_NUM_CONV):
//		num_conv = *(usb_msg_trx+PARAM_POS);
		break;
	case(USB_SET_ADC_DATA_RATE):
		adc_data_rate = *(usb_msg_trx+PARAM_POS);
		break;
	case(USB_SET_DEV_STATE):
		dev_state_old = *(usb_msg_trx + PARAM_POS);	// Device will return to this state after disconnecting from PC
		break;
	case(USB_SET_SYSOCAL_EN):
		sysocal_enable = *(usb_msg_trx+PARAM_POS);
		break;
	case(USB_SET_DELAY_EN):
		delay_enable = *(usb_msg_trx+PARAM_POS);
		break;
	case(USB_SET_MEAS_RATE):
		long_int_merge(usb_msg_trx+PARAM_POS,&meas_rate,1);	// 1 is the number of uint32 values to convert - only meas_rate
		break;
	case(USB_SET_MEAS_START):
		start_immediately = *(usb_msg_trx+PARAM_POS);
		start_zero = *(usb_msg_trx+PARAM_POS+1);
		break;
	case(USB_SET_KEEP_DATA):
		KeepData = *(usb_msg_trx+PARAM_POS);
		break;
	case(USB_SET_CAL_EN):
		CalEn = *(usb_msg_trx+PARAM_POS);
		break;
	}

	usb_ack();
}


void usb_set_adc_vrefcon (void)
{
	uint8_t status = 0;

	adc_mux1_vrefcon = *(usb_msg_trx+PARAM_POS);

	// Since this parameter is not set through adc_driver functions, set it in this function.
	// Valid until next power cycle or reset
	adc_mux1_register adc_mux_1 =
	{
		adc_mux_1.adc_clkstat = MUX1_CLKSTAT_INT_OSC,
		adc_mux_1.adc_vrefcon = *(usb_msg_trx+PARAM_POS),
		adc_mux_1.adc_refselt = MUX1_REFSELT_INT_REF0,
		adc_mux_1.adc_muxcal = MUX1_MUXCAL_GAIN_CAL
	};

	adc_mux1_ptr = &adc_mux_1;

	adc_spi_define_wreg_mux1(adc_mux1_ptr);

	adcWorkInProgress = TRUE;

	P9OUT |= ADC_START;

	do
	{
		status = adc_spi_wreg_single();
	}
	while (status == STATUS_FAIL);

	P9OUT &= ~ADC_START;

	LPM0;

	adcWorkInProgress = FALSE;

	usb_ack();
}


void usb_set_rtc(void)
{
	// See page 600 msp430 UG for writing to RTC when it runs:
	// Takes effect immediately, RTC stops, RT1PS resets

	RTCCTL1 |= 0x40;	// If yes, disable it to configure new clock value

	RTCYEAR = (uint16_t)(*(usb_msg_trx+RTC_YEAR_POS));	// uint16_t, use cast
	RTCMON = *(usb_msg_trx+RTC_MON_POS);
	RTCDAY = *(usb_msg_trx+RTC_DAY_POS);
	RTCHOUR = *(usb_msg_trx+RTC_HOUR_POS);
	RTCMIN = *(usb_msg_trx+RTC_MIN_POS);
	RTCSEC = *(usb_msg_trx+RTC_SEC_POS);

//	RTCPS1CTL |= RT1IP__256;	// Set prescaler to generate 0.5 Hz intervals -> moved to init.c

	RTCCTL1 &= ~0x40;		// Release RTC for operation RTCHOLD is 0x4000 but it didn't work with this value

//	clear_lockback_batbackup();

	usb_ack();
}

