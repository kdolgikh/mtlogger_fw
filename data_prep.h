/*
 * data_prep.h
 *
 *  Created on: 02/01/2017
 *      Author: kdolgikh
 */

#ifndef DATA_PREP_H_
#define DATA_PREP_H_

#include "stdint.h"

#define CRC_NUM_BYTES   2   // Number of bytes after CRC value uint16t is divided into two values uint8_t

/**
 * \brief Divide uint16_t into two uint8_t and store into array pointed by data8_ptr
 *
 * @param	data16		uint16_t variable to be divided
 * @param	*data8_ptr	Pointer to the array for the result
 */
void int_divide (uint16_t data16, uint8_t *data8_ptr);

/**
 * \brief Merge two uint8_t into uint16_t
 *
 * @param	*data8_ptr	pointer to the array of two uint8_t variables
 * @return				Return uint16_t variable
 */
uint16_t int_merge (uint8_t *data8_ptr);


/**
* \brief Divides uint32_t array into uint8_t array
*
* @param *data32_ptr	Ptr to the input array of uint32_t
* @param *data8_ptr		Ptr to the output array of uint8_t
* @param data32_length	Length of the uint32_t array
*/
void long_int_divide (uint32_t *data32_ptr, uint8_t *data8_ptr, uint8_t data32_length);


/**
* \brief Gets uint8_t array and merges it into uint32_t array
*
* @param *data8_ptr		Ptr to the input array of uint8_t (source)
* @param *data32_ptr	Ptr to the output array of uint32_t (destination)
* @param data32_length	Length of the uint32_t array
*/
void long_int_merge (uint8_t *data8_ptr, uint32_t *data32_ptr, uint8_t data32_length);


//Reverse order is used because data in flash (and hence SD-card) is uint32
void reverse_order (uint8_t *data_source_stg, uint8_t *data_dest_stg, uint16_t size);

#endif /* DATA_PREP_H_ */
