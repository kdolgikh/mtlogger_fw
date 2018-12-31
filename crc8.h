/*
 * crc8.h
 *
 *  Created on: Apr 1, 2017
 *      Author: kdolgikh
 */

#ifndef CRC8_H_
#define CRC8_H_

#include "stdint.h"
#include "adc_driver.h"

#define CRC_BYTES		2		// number of CRC bytes added to the block of 170 conversions
#define BAD_DATA		0xBD	// BD stands for "bad data"
#define CORRECT_DATA	0xCD	// CD stands for "correct data"


/**
 * \brief Generates CRC signature for each 170 conversion result block
 *
 * @param	input_data_ptr	Input data to generate CRC signatures
 * @param   data_length     Length of the data
 * @return	crc_result		Returns CRC result of a block
 */
uint16_t crc8_set (uint8_t *input_data_ptr, uint16_t data_length);


/**
 * \brief Checks CRC of conversion results from flash against the stored CRC signatures
 *
 * @param	crc_result		The stored CRC signature
 * @param	input_data_ptr	Input data to generate CRC signatures to compare with stored signatures
 * @param   data_length     Length of the data
 * @return	status			Returns GOOD_DATA or BAD_DATA. Return value for each block is sent to MATLAB
 */
uint8_t crc8_check (uint16_t crc_result, uint8_t *input_data_ptr, uint16_t data_length);


void check_data_crc(uint8_t *storage);

// No need to check CRC in RAM since it hasn't been written to flash or SD-card
void replace_ram_stg_crc(uint8_t *storage, uint16_t index);

#endif /* CRC8_H_ */
