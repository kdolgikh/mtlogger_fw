/*
 * measure.c
 *
 *  Created on: Dec 19, 2017
 *      Author: kdolgikh
 */

#include "driverlib.h"
#include "sleep_timer.h"
#include "adc_driver.h"
#include "sdcard.h"
#include "crc8.h"
#include "data_prep.h"
#include "mux_adg706_driver.h"
#include "measure.h"
#include "real_time_clock.h"
#include "send_over_usb.h"

uint32_t num_conv = NUM_CONV;
uint32_t meas_rate = 0;

uint16_t crc_value = 0;

uint8_t *flash_data8_ptr = (uint8_t *)BANK1_START;
uint32_t *flash_data32_ptr = (uint32_t *)BANK1_START;
uint16_t index = TS_W_CRC_NUM_BYTES;
uint8_t ram_stg[SEGMENT_SIZE] = {0};                                    // RAM storage for conversion results of all channels
uint32_t ram_stg_32[RAM_STG_32_LGTH] = {0};                             // RAM storage for conv results in uint32_t format
uint16_t segment_count = 0;												// Counts number of segments to be written to flash
uint32_t num_segments = 0;												// Counts total number of segments written to both flash and SD-card
uint8_t sdc_full = FALSE;                                               // Flag indicating that SD-card is full
uint8_t mem_full = FALSE;												// Flag indicating that device memory is full
uint32_t num_segments2sdc = NUM_SEGMENTS_FLASH;

extern uint32_t max_segments_sd;

uint8_t firstMeasurement = FALSE;
uint8_t newSegment = TRUE;

uint8_t KeepData = FALSE;
uint8_t CalEn = FALSE;

exit_point usb_exit;

/*device_state measure (uint8_t channel_num,
                      uint32_t num_conv) // do I need to add here meas rate?
{
    device_state state;

    if (channel_num != ALL_CHANNELS)
        state = adc_measure_single_channel(uint8_t channel_num,
                                                uint32_t num_conv);
    else
        state = adc_measure_all_channels(void);

    return state;
}


device_state adc_measure_single_channel(uint8_t channel_num,
                                        uint32_t num_conv)
{
    state = switch_mux_ch(channel_num);
    if (state == parameter_error)
        return state;
}*/


device_state measure_all_channels_store_flash_sdc (void)
{
    while (1) //repeat until device memory is full
    {
        while (segment_count < NUM_SEGMENTS_FLASH) // repeat until flash is full
        {
        	while (index < SEGMENT_SIZE - NUM_SPARE_BYTES) // repeat until RAM storage is full, then write to flash.
            {
        		// Checks cases of USB connection:
            	// 1) USB was connected during measurement when RAM stg didn't become full
            	// 2) USB was connected during flash write when flash didn't become full
            	// 3) USB was connected during SD-card write
            	if(!USBconnected)
        		{
					if(firstMeasurement)
						firstMeasurement = FALSE;
					else
					{
						LPM3;

						if(USBconnected)			// If the device was woken up by USB connection and not meas interval expiration
						{
							usb_exit = exit5;
							return usb_connected;
						}
					}

            		if(newSegment)
            		{
						gen_time_stamp(ram_stg);	// insert time stamp into ram_stg for the first measurement in each segment
						newSegment = FALSE;
            		}
        		}
            	else
            	{
            		usb_exit=exit4;
            		return usb_connected;
            	}

            	if (CalEn)
            	{
					adc_calibration(adc_data_rate);
					adc_setup(adc_data_rate, adc_mux1_vrefcon);
            	}

            	index = adc_measure_all_channels(ram_stg, index, &crc_value);
            }

            newSegment = TRUE;															// Every segment should have a time stamp

            // Checks cases of USB connection:
            // 1) USB was connected during the last conversion of the segment (the code didn't enter while(index..) loop
            if(USBconnected)
            {
            	usb_exit=exit6;
            	return usb_connected;
            }

            index = TS_W_CRC_NUM_BYTES;													// Reinit index for the next loop run

            // If connected to USB before the segment was written to flash, it will be retrieved directly from RAM,
            // so no need to write the last segment to flash and account it as being written to flash and later SD-card
            segment_count++;				// this value is reinit every time it count up to NUM_SEGMENTS_FLASH
            num_segments++;					// this value shows total num of segments written to both flash and SD-card

            write_data2flash();

			if (num_segments == max_segments_sd)                                // Last segment that can be written to SD-card until it's full
			{
			 num_segments2sdc = (uint32_t)(flash_data8_ptr - BANK1_START) >> 9; // Divide by 512 (or right shift by 9) to get number of segments
																				// to write to SD-card.
			 break; 															// Break while (segment_count ...) loop
			}
         }

        segment_count = 0;											// Reinit segment count for the next while(segment_count ...) run

        if (sdc_full)
            break;  // break while(1) loop                          // Do not write to SD-card if it's full

        // Checks cases of USB connection:
        // 1) USB was connected during flash write when it became full (the code didn't enter while(segment_count...) loop)
        // Program exits with end address of flash Bank 3, which indicates that flash has been filled with data,
        // or with flash address where it has stopped due to max_segments_sd condition.
        if(USBconnected)
        {
        	usb_exit=exit7;
        	return usb_connected;
        }

        flash_data8_ptr = (uint8_t *)BANK1_START;                  // Reinit flash ptr to start writing to SD-card and flash from Bank 1
        flash_data32_ptr = (uint32_t *)BANK1_START;                // Reinit flash ptr to start writing to flash in the next loop from Bank 1

        write_data2sdc(sdc_block_address, flash_data8_ptr, num_segments2sdc);

        if (num_segments != max_segments_sd)
        	sdc_block_address += (uint32_t)SDC_ADDR_INCREMENT;			// Points to the block to start writing next time, in bytes
        else
        {
        	sdc_block_address += (num_segments2sdc << 9);		// The last addr points to the end of SD-card, which is used in send_conv_result
        	sdc_full = TRUE;
        }
    }

    RTCPS1CTL &= ~RT1PSIE; // disable interrupt to turn off measure intervals
    // Once done testing, turn of RTC clock here as well as you will not need it

    mem_full = TRUE;

    if(USBconnected)
    {
    	usb_exit=exit8;
    	return usb_connected;
    }
    else
    	return dev_mem_full;
}

void write_data2flash (void)
{
    uint8_t status = 0;

    long_int_merge(ram_stg,ram_stg_32,RAM_STG_32_LGTH);                      // Merge values in RAM storage into uint32_t variables
                                                                             // Writing uint32t is 2x faster than writing uint8_t/uint16_t
    // write RAM storage data to flash
     do
     {
         FlashCtl_eraseSegment(flash_data8_ptr);
         status = FlashCtl_performEraseCheck(flash_data8_ptr,SEGMENT_SIZE);  // Check that segment contains only 0xFF
     }
     while(status == STATUS_FAIL);
     FlashCtl_write32(ram_stg_32,flash_data32_ptr,RAM_STG_32_LGTH);

     flash_data8_ptr += SEGMENT_SIZE;                                        // Increase flash8 pointer to point to next segment
     flash_data32_ptr += RAM_STG_32_LGTH;                                    // Increase flash32 pointer to point to next segment

}

void write_data2sdc (uint32_t block_addr, uint8_t *data_ptr, uint32_t num_segments)
{

	sdcPowerOn();

    if (sdcStart() == SDC_SUCCESS)                           	// Set SD-card in SPI-mode
        sdc_SPI_Init();                                         // Increase clock frequency to 20 MHz

    sdc_response = sdcWriteMultipleBlocks(block_addr,    		// Start from this block
                                         data_ptr,		   	    // Write this data
                                         num_segments);     	// Write this number of segments
    sdcPowerOff();
}

void mem_counters_cleanup(void)
{
	index = TS_W_CRC_NUM_BYTES;
	segment_count = 0;
	num_segments = 0;
    flash_data8_ptr = (uint8_t *)BANK1_START;
    flash_data32_ptr = (uint32_t *)BANK1_START;
	num_segments2sdc = NUM_SEGMENTS_FLASH;
	sdc_block_address = 0;
}
