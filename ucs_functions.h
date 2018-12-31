/*
 * ucs_functions.h
 *
 *  Created on: Nov 30, 2016
 *      Author: kdolgikh
 */

/** \file ucs_functions.h
 * 	\brief Contains a set of UCS functions
 */

#ifndef UCS_FUNCTIONS_H_
#define UCS_FUNCTIONS_H_

#include <stdint.h>

#define UCS_STATUS_OK     0
#define UCS_STATUS_ERROR  1

/** \brief Enable XT1 crystal and RTC
 *
 *	This function is used on device startup.
 *	DCO fault flag will also appear at this time.
 *
 * 	Execution flow:
 * 	1. XT1 is enabled by default after reset. Wait until XT1 settles.
 * 	2. Set XT1 drive to the smallest allowed value.
 */
void enable_XT1_RTC (void);


/** \brief Clear XT1 related oscillator fault flags
 *
 *	This function is used whenever it's necessary to clear XT1 faults
 */
void clear_XT1_OFFG (void);


/** \brief Clear XT1 related oscillator fault flags with a specified timeout
 *
 *	This function is used whenever it's necessary to clear XT1 faults with the ability
 *	to exit the function upon timeout expiration
 *
 *	@param 	timeout 	Amount of time to try clearing oscillator fault flag
 *	@return 			Return status of the clear operation: success or fail
 */
uint8_t clear_XT1_OFFG_timeout (uint32_t timeout); //!!! uint32_t may be changed to uint_16t !!!//


/** \brief Disable XT1
 *
 *	This function is used whenever it's necessary to disable XT1
 */
void disable_XT1 (void);


/** \brief Clear LOCKBACK flag of the battery backup system
 *
 *	Until cleared, battery backup system doesn't allow XT1 to operate
 */
void clear_lockback_batbackup (void);


/** \brief Enable XT2 crystal
 *
 *	This function is used when USB VBUS appears
 *
 * 	Execution flow:
 * 	1. Enable XT2 and wait until it settles.
 * 	2. Set XT2 drive to the smallest allowed value.
 */
void enable_XT2 (void);


/** \brief Clear XT2 related oscillator fault flags
 *
 *	This function is used whenever it's necessary to clear XT2 faults
 */
void clear_XT2_OFFG (void);


/** \brief Clear XT2 related oscillator fault flags with a specified timeout
 *
 *	This function is used whenever it's necessary to clear XT1 faults with the ability
 *	to exit the function upon timeout expiration
 *
 *	@param 	timeout 	Amount of time to try clearing oscillator fault flag
 *	@return 		 	Return status of the clear operation: success or fail
 */
uint8_t clear_XT2_OFFG_timeout (uint32_t timeout); //!!! uint32_t may be changed to uint_16t !!!//


/** \brief Disable XT2
 *
 *	This function is used whenever it's necessary to disable XT2
 */
void disable_XT2 (void);

#endif /* UCS_FUNCTIONS_H_ */
