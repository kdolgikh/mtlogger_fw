/*
 * data_prep.c
 *
 * Created on: 02/01/2017
 *      Author: kdolgikh
 */
#include "stdint.h"
#include "data_prep.h"


void int_divide (uint16_t data16, uint8_t *data8_ptr)
{
	*data8_ptr = (data16 >> 8) & 0xFF;
	*(data8_ptr + 1) = data16 & 0xFF;
}


uint16_t int_merge (uint8_t *data8_ptr)
{
	uint16_t temp0 = 0;
	uint16_t temp1 = 0;
	uint16_t temp2 = 0;

	temp0 = *data8_ptr;
	temp1 = *(data8_ptr+1);
	temp2 = (temp0 << 8) | temp1;

	return temp2;
}


void long_int_divide (uint32_t *data32_ptr, uint8_t *data8_ptr, uint8_t data32_length)
{
	uint8_t i;
	uint16_t k = 0;

	for (i=0; i < data32_length; i++)
	{
		*(data8_ptr+k)	 = 	(*(data32_ptr+i) >> 24) & 0xFF;		//MSB
		*(data8_ptr+k+1) = 	(*(data32_ptr+i) >> 16) & 0xFF;
		*(data8_ptr+k+2) = 	(*(data32_ptr+i) >> 8) & 0xFF;
		*(data8_ptr+k+3) = 	*(data32_ptr+i) & 0xFF;				//LSB
		k+=4;													// be sure that data32 length equals 1/4*data8 length
	}
}


void long_int_merge (uint8_t *data8_ptr, uint32_t *data32_ptr, uint8_t data32_length)
{
	uint8_t i;
	uint16_t k = 0;
	uint32_t temp0 = 0;
	uint32_t temp1 = 0;
	uint32_t temp2 = 0;
	uint32_t temp3 = 0;

	for (i=0; i < data32_length; i++)
	{// temporary variables are used instead of cast operator
		temp0 = *(data8_ptr+ k); 	// MSB
		temp1 = *(data8_ptr+k+1);
		temp2 = *(data8_ptr+k+2);
		temp3 = *(data8_ptr+k+3);	// LSB
		*(data32_ptr+i) = (temp0<<24) | (temp1<<16) | (temp2<<8) | temp3;
		k+=4;						// be sure that data32 length equals 1/4*data8 length
	}
}

void reverse_order (uint8_t *data_source_stg, uint8_t *data_dest_stg, uint16_t size)
{
	uint16_t i=0;
	uint16_t j=0;

	while (i < size)
	{
		*(data_dest_stg+i) = *(data_source_stg+j+3);
		*(data_dest_stg+i+1) = *(data_source_stg+j+2);
		*(data_dest_stg+i+2) = *(data_source_stg+j+1);
		*(data_dest_stg+i+3) = *(data_source_stg+j);
		i+=4;
		j+=4;
	}
}
