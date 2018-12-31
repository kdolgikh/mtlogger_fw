/*
 * measure.h
 *
 *  Created on: Dec 19, 2017
 *      Author: kdolg
 */

#ifndef MEASURE_H_
#define MEASURE_H_

#include "stdint.h"
#include "states.h"
#include "crc8.h"
#include "real_time_clock.h"

/// \name Memory related definitions
//@{
#define BANK1_START             0x28000     // Start address of Bank 1
#define BANK2_START             0x48000     // Start address of Bank 2
#define BANK3_START             0x68000     // Start address of Bank 3
#define SEGMENT_SIZE            512         // Main flash memory segment size, byte
#define NUM_SEGMENTS_FLASH      768         // Number of 512 Byte segments in 384 KB flash
#define SDC_ADDR_INCREMENT		NUM_SEGMENTS_FLASH*SEGMENT_SIZE		// Address increment when writing flash to sdc
#define RAM_STG_32_LGTH         (SEGMENT_SIZE>>2)         // Ram storage contains 128 uint32t values
//@}


#define NUM_CONV                10          // Number of times to measure all channels
#define SINGLE_CONV             1           // Single conversion
#define NUM_SPARE_BYTES         5           // Number of spare bytes in RAM storage of SEGMENT_SIZE
                                            // left after writing TS (5 Bytes) with CRC (2 Bytes)
											// and 10 measurement results for 16 channels with CRC (500 Bytes).

#define CONV_LENGTH_16CH        48           // Length of conversion results for 16 channels

// Misc cumulative length of different field in storage segment of 512 bytes
#define CONV_CRC_LGTH				(CONV_LENGTH_16CH +CRC_BYTES)						// 50
#define TS_CRC_CONV_BYTES			(TS_NUM_BYTES +CRC_BYTES +CONV_LENGTH_16CH)			// 55
#define TS_CRC_CONV_CRC_SP_BYTES	(TS_CRC_CONV_BYTES +CRC_BYTES +NUM_SPARE_BYTES)		// 62

extern uint8_t firstMeasurement;
extern uint32_t meas_rate;

extern exit_point usb_exit;

/**
 *  Number of times to measure all channels
 */
extern uint32_t num_conv;

/**
 * Array in RAM to hold conversion results for all channels
 */
extern uint8_t ram_stg [SEGMENT_SIZE];


/**
 * RAM storage for conv results in uint32_t format
 */
extern uint32_t ram_stg_32[RAM_STG_32_LGTH];

extern uint16_t index;

extern uint8_t *flash_data8_ptr;        // Pointer used to erase flash segments and for SD-card write
extern uint32_t *flash_data32_ptr;      // Pointer used to write 32 bit conv results with CRC into flash

extern uint8_t sdc_full;
extern uint8_t mem_full;

extern uint32_t num_segments2sdc;

extern uint8_t KeepData;
extern uint8_t CalEn;

device_state adc_measure_single_channel(uint8_t channel_num,
                                        uint32_t num_conv);


// The function's organization allows writing entire flash once SD-card is full
device_state measure_all_channels_store_flash_sdc (void);


void write_data2flash (void);

void write_data2sdc (uint32_t block_addr, uint8_t *data_ptr, uint32_t num_segments);

void mem_counters_cleanup (void);

/*device_state measure (uint8_t channel_num,
                      uint32_t num_conv);*/


/*
*
 *  \brief  Measure single channel 170 x number_of_conv and store to flash
 *
 *  This function is needed to determine the measurement noise when
 *  measuring many samples for a single channel.
 *  Max occupied flash is 255 x 512 = 127.5 KB
 *
 *  @param channel_number       ADC channel number
 *  @param  num_conv            Number of conversions; max is 255. This yields 255 * 170 = 43350 meas
 *                              per single channel
 *  @param  *ram_stg            adc_ram_storage of a channel
 *  @param  index               Position in the adc_ram_storage to stroe current measurement
 *  @return device_state        Returns adc_ready or mcu_broken if flash erasing failed

device_state adc_measure_store2flash (uint8_t channel_number, uint8_t num_conv, uint8_t *ram_stg);


*
 *  \brief Measure all channels and store to flash
 *
 *  @param num_conv_all         Number of measurement blocks of 170 measurements x 16 channels
 *                              Represents the same idea as number_of_conv but expanded for all 16 channels.
 *                              Add possibility to set it through CLI.
 *                              This number should be smaller or equal to num_flash_full.

device_state adc_measure_all_store2flash (uint8_t num_conv);


*
 * \brief Measure all channels, write to flash and SD-card
 *
 *  Use this function when need to measure all channels more than num_flash_full times
 *  1 < num_flash_blocks (384KB) <  sdc_max_flash_blocks
 *
 *
 * @param   flash_blocks_384KB  Number of flash block (384 KB) that can be written to SD-card.
 *                              Should not be bigger than max_num_flash_blocks.
 * @return  device_state        Returns device state

device_state measure_all_flash_sdc (uint16_t flash_blocks_384KB);
*/


#endif /* MEASURE_H_ */
