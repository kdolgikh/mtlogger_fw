/*
 * real_time_clock.c
 *
 *  Created on: Dec 28, 2017
 *      Author: kdolgikh
 */
#include "driverlib.h"
#include "real_time_clock.h"
#include "data_prep.h"
#include "crc8.h"
#include "measure.h"
#include "sleep_timer.h"

uint8_t rtc_sec = 0;	// rw
uint8_t rtc_min = 0;	// rw
uint8_t rtc_hour = 0;	// rw
uint8_t rtc_day = 0;	// Day of month, rw
uint8_t rtc_mon = 0;	// Month, rw
uint16_t rtc_year = 0;	// rw

uint32_t time_stamp = 0;
volatile uint32_t rt1ps_count = 0;			// counts num of RT1IP interrupts
uint8_t start_immediately = FALSE; 	// start immediately after disconnection
uint8_t start_zero = FALSE;  		// start at 00 clock (00 sec, xx min 00 sec, xx hours 00 min 00 sec, etc)

uint16_t ts_crc = 0;	// time stamp CRC

uint8_t ts_debug_array[7]={0};

void gen_time_stamp (uint8_t *ts_storage_ptr)
{
uint8_t i;

	/*
	uint32_t time_stamp = 0;
	uint16_t ts_crc = 0;	// time stamp CRC
*/
//	uint8_t ts_ready = FALSE;

	// Reading from RTC registers is implemented this way in order not to wait 1 sec until it's safe to read
	// upon RTCRDYIFG. Reading by simply checking RTCRDY once is not safe because it may become 0 any time
	// There's also a chance that RTCRDY will become low after checking that it's High and before reading RTC value,
	// the code below doesn't remedy this.
/*	while(!ts_ready)
	{
		if (RTCCTL1 & 0x10)
		{
			rtc_year = RTCYEAR;
			if (RTCCTL1 & 0x10)
			{
				rtc_mon = RTCMON;
				if (RTCCTL1 & 0x10)
				{
					rtc_day = RTCDAY;
					if (RTCCTL1 & 0x10)
					{
						rtc_hour = RTCHOUR;
						if (RTCCTL1 & 0x10)
						{
							rtc_min = RTCMIN;
							if (RTCCTL1 & 0x10)
							{
								rtc_sec = RTCSEC;
								ts_ready = TRUE;
							}
							else
								while (!(RTCCTL1 & 0x10))
									;
						}
						else
							while (!(RTCCTL1 & 0x10))
								;
					}
					else
						while (!(RTCCTL1 & 0x10))
							;
				}
				else
					while (!(RTCCTL1 & 0x10))
						;
			}
			else
				while (!(RTCCTL1 & 0x10))
					;
		}
		else
			while (!(RTCCTL1 & 0x10))
				;
	}*/

	while (!(RTCCTL1 & 0x10)) 	// If RTCRDY (0x10) is LOW (registers are being updated), wait
		;

	rtc_year = RTCYEAR;			// Retrieve TS data. Could do it inside RTC ISR
	rtc_mon = RTCMON;
	rtc_day = RTCDAY;
	rtc_hour = RTCHOUR;
	rtc_min = RTCMIN;
	rtc_sec = RTCSEC;

	// Year will be in the format 18-255 for 2018-2255 accordingly (to meet uint8_t)
	// Store Year in ts_storage
	*(ts_storage_ptr) = (uint8_t)rtc_year;

	time_stamp = ((uint32_t)rtc_mon)<<28		// Generate MDHMS time stamp
				 | ((uint32_t)rtc_day)<<23
				 | ((uint32_t)rtc_hour)<<18
				 | ((uint32_t)rtc_min)<<12
				 | ((uint32_t)rtc_sec)<<6;		// 6 Bits are unused

	// Divide time_stamp into 4 uint8_t and store in ts_storage
	long_int_divide(&time_stamp, (ts_storage_ptr+TS_BYTES_POSITION), 1);

	// Generate CRC for Year + MDHMS
	ts_crc = crc8_set(ts_storage_ptr,TS_NUM_BYTES);

	// Divide CRC into 2 uint8_t and store in ts_storage
	int_divide(ts_crc,(ts_storage_ptr + TS_CRC_BYTE1_POS));


	for (i=0;i<7;i++)
		*(ts_debug_array+i) = *(ts_storage_ptr+i);
}

