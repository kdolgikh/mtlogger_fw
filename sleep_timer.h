/*
 * sleep_timer.h
 *
 *  Created on: Apr 9, 2017
 *      Author: kdolgikh
 */

#ifndef SLEEP_TIMER_H_
#define SLEEP_TIMER_H_

#include <stdint.h>

#define TRUE	1
#define FALSE	0

extern volatile uint16_t Fast_Timer_Rollover_Count;
extern volatile uint8_t timer_event;
extern volatile uint8_t rtc_event;
extern uint8_t delay_enable;

/**
 *	\brief	Timer that puts device into sleep for num_rollovers * 128 seconds
 *
 *	@param	num_rollovers	Number of timer rollovers to sleep for
 */
//void Sleep_Timer_Slow(uint16_t num_rollovers);


/**
 * \brief 	Puts MCU to sleep for the specified number of ACLK cycles.
 *
 * @param	cycles	The number of cycles to sleep for, in terms of ACLK ticks
 */
void Sleep_Timer_Fast(uint32_t cycles);

#endif /* SLEEP_TIMER_H_ */
