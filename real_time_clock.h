/*
 * real_time_clock.h
 *
 *  Created on: Dec 28, 2017
 *      Author: kdolgikh
 */

#ifndef REAL_TIME_CLOCK_H_
#define REAL_TIME_CLOCK_H_

#include "stdint.h"

//#define TS_YEAR_BYTE_POSITION	0	// Position in RAM storage to write 1 Byte of Year
#define TS_BYTES_POSITION		1	// Position in RAM storage to start writing 4 Bytes of M-D-H-Min-Sec
#define TS_CRC_BYTE1_POS		5	// Position in RAM storage to start writing 2 Bytes of TS CRC
#define TS_CRC_BYTE2_POS		6	// Position of the second TS CRC byte

#define TS_YEAR_LGTH			1	// Number of bytes in Year
//#define TS_MDHMS_LGTH			4	// Number of bytes in M-D-H-Min-Sec
#define TS_NUM_BYTES			5	// Number of bytes of TS (Year + MDHMS)
#define TS_W_CRC_NUM_BYTES		(TS_NUM_BYTES +CRC_BYTES)	// 7

extern uint8_t rtc_sec;
extern uint8_t rtc_min;
extern uint8_t rtc_hour;
extern uint8_t rtc_day;
extern uint8_t rtc_mon;
extern uint16_t rtc_year;

extern uint32_t time_stamp;
extern volatile uint32_t rt1ps_count;
extern uint8_t start_immediately;
extern uint8_t start_zero;

void gen_time_stamp(uint8_t *ts_storage_ptr);


#endif /* REAL_TIME_CLOCK_H_ */
