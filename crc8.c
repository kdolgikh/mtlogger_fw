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
//******************************************************************************
/*
 * crc8.c
 *
 *  Created on: Apr 1, 2017
 *      Author: kdolgikh
 *      Based on the TI driverlib
 */
#include "stdint.h"
#include "crc8.h"
#include "driverlib.h"
#include "measure.h"
#include "data_prep.h"
#include "real_time_clock.h"


uint16_t crc8_set (uint8_t *input_data_ptr,
                   uint16_t data_length)
{
	uint16_t crc_seed = 0xDEAD;
	uint16_t crc_result;
	uint16_t i;

	CRC_setSeed(CRC_BASE, crc_seed);

	for(i = 0; i < data_length; i++)
	{
		CRC_set8BitData(CRC_BASE,*(input_data_ptr+i));	//Add data into the CRC signature
	}

	crc_result = CRC_getResult(CRC_BASE);

	return crc_result;
}

uint8_t crc8_check (uint16_t crc_result,
                    uint8_t *input_data_ptr,
                    uint16_t data_length)
{
	uint16_t crc_seed = 0xDEAD;
	uint16_t i;

	CRC_setSeed(CRC_BASE, crc_seed);

	for(i = 0; i < data_length; i++)
	{
		CRC_set8BitData(CRC_BASE,*(input_data_ptr+i));
	}

	if(crc_result == CRC_getResult(CRC_BASE))
		return CORRECT_DATA;
	else return BAD_DATA;
}


void check_data_crc(uint8_t *storage)
{
	uint16_t crc_val = 0;
	uint16_t i = 0;

	// Check time stamp CRC
	crc_val = int_merge(storage + TS_CRC_BYTE1_POS);
	*(storage + TS_CRC_BYTE1_POS) = crc8_check(crc_val,storage,TS_NUM_BYTES);
	*(storage + TS_CRC_BYTE2_POS) = 0; 				// set the second byte of CRC to 0

	// Check conv results CRC
	while (i <= (SEGMENT_SIZE - TS_CRC_CONV_CRC_SP_BYTES))
	{
		crc_val = int_merge(storage +TS_CRC_CONV_BYTES +i);

		*(storage + TS_CRC_CONV_BYTES + i) = crc8_check(crc_val,
						   	   	   	   	   	   	   	    (storage +TS_W_CRC_NUM_BYTES +i),
														CONV_LENGTH_16CH);

		*(storage + TS_CRC_CONV_BYTES + i + 1) = 0; // set the second byte of CRC to 0

		i += CONV_CRC_LGTH;
	}
}

void replace_ram_stg_crc(uint8_t *storage, uint16_t index)
{
	uint16_t i = TS_CRC_CONV_BYTES;

	// Replace TS CRC with "correct data" token
	*(storage + TS_CRC_BYTE1_POS) = CORRECT_DATA;
	*(storage + TS_CRC_BYTE2_POS) = 0;

	// Replace conv results CRC with "correct data" token
	while (i <= (index - CRC_BYTES))
	{
		*(storage + i) = CORRECT_DATA;
		*(storage + i + 1) = 0;
		i += CONV_CRC_LGTH;
	}
}
