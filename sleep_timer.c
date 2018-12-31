/*
 * sleep_timer.c
 *
 *  Created on: Apr 9, 2017
 *      Author: kdolgikh
 */

#include "driverlib.h"
#include <stdint.h>
#include "sleep_timer.h"
#include "states.h"
/*#include "mux_adg706_driver.h"	// For DEBUG only, remove later*/

//volatile uint16_t Fast_Timer_Rollover_Count = 0;
volatile uint8_t timer_event = FALSE;

uint8_t delay_enable = FALSE;

/*void Sleep_Timer_Slow(uint16_t num_rollovers)
{
	// 1 overflow is 64 seconds

	uint16_t i;
	timer_event = TRUE;

  disable_mux_adg706();

	P4OUT ^= A0;	// for DEBUG only, remove when not needed

	TA1CTL |= MC__CONTINUOUS;		//Start timer

	for (i = 0; i < num_rollovers && timer_event; i++)
	{	timer_event = FALSE;
		LPM3;
	}

	TA1CTL ^= MC__CONTINUOUS;		// Stop timer
	TA1R = 0;						// Clear TA1R

	P4OUT ^= A0;	// for DEBUG only, remove when not needed

}*/

void Sleep_Timer_Fast(uint32_t cycles)
{
	// 1 cycle = 1/4096 or 0.244 ms or 4882 MCLK cycles.
	// If MCU wake-up time is slow (165 us), having 1 cycle of smaller than 165 us is not reasonable, use _delay_cycles(x);
	// account for extra 165 us required to wake-up, if necessary.
	// If MCU wake-up time is fast (3 us), TA2 source frequency can be increased to 32768 (1 cycle is 30.5 us)
	// or even changed to SMCLK if smaller than 30.5 us sleep duration is required.

	uint16_t i;
	uint32_t rollovers;
	uint16_t Sleep_Cycles;

/*	disable_mux_adg706();*/

	rollovers = (cycles >> 16);			// Upper word is number of rollovers
	Sleep_Cycles = (cycles & 0xFFFF); 	// Lower word is number of cycles to run

	if (Sleep_Cycles == 0)		 	// If 0 (if rollovers >0), then TAIFG and CCRIFG will occur simultaneously,
		Sleep_Cycles = 1;			// and, because CCRIFG has higher priority, the rollover count will not be set
									// before evaluating the for loop. As the result the device will sleep for 1 rollover longer.
									// When sleep_cycles = 1, CCRIFG will occur later than TAIFG. Also, protection from 0 input.

	TA2CCR1 = Sleep_Cycles; 		// Set the CCR to wait the number of cycles given

/*	P4OUT ^= A0;	// for DEBUG only, remove when not needed*/

	TA2CTL |= MC__CONTINUOUS;		//Start timer

	LPM3;

	for (i = 0; i < rollovers && timer_event; i++)
	{
		timer_event = FALSE;
		LPM3;
	}

	TA2CTL ^= MC__CONTINUOUS;		// Stop timer
	TA2R = 0;						// Clear TA1R

/*	P4OUT ^= A0;	// for DEBUG only, remove when not needed*/
}
