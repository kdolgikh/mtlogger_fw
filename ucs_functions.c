/*
 * ucs_functions.c
 *
 *  Created on: Nov 30, 2016
 *      Author: kdolgikh
 */

/** \file ucs_functions.c
 * 	\brief Contains a set of UCS functions
 */

#include "msp430f5659.h"
#include "ucs_functions.h"

void enable_XT1_RTC (void)	// RTC and XT1 should be set up before clearing LOCKBACK, see page 130 UG
{
	RTCPS1CTL |= RT1IP__256;									// Set RTC prescaler to generate 0.5 Hz intervals
	clear_XT1_OFFG();											// Allow XT1 to settle
	UCSCTL6 &= ~XT1DRIVE_3;										// Set the lowest drive setting for 32.768 Hz crystal
}


void clear_XT1_OFFG (void)
{
    do
    {
    	UCSCTL7 &= ~(XT1LFOFFG + DCOFFG); 						// Clear XT1 and DCO fault flags
    	SFRIFG1 &= ~OFIFG; 										// Clear oscillator fault flag, see page 82 UG
    }
    while (SFRIFG1&OFIFG);										// Loop until OFIFG is cleared
}


uint8_t clear_XT1_OFFG_timeout (uint32_t timeout)
{
    do {
        UCSCTL7 &= ~(XT1LFOFFG + DCOFFG);                       // Clear XT1 and DCO fault flags
        SFRIFG1 &= ~OFIFG;										// Clear oscillator fault flag, see page 82 UG
    } while ((SFRIFG1&OFIFG) && --timeout);						// Do until timeout has expired or flag has been cleared

    if (timeout)
        return (UCS_STATUS_OK);									// Timeout has not expired
    else
        return (UCS_STATUS_ERROR);								// Timeout has expired
}


void disable_XT1 (void)
{
    UCSCTL6 |= XT1OFF;                                          // Disable XT1
}


void clear_lockback_batbackup (void)
{
	while (BAKCTL&LOCKBAK)
	{
		BAKCTL &= ~LOCKBAK;										// Clear LOCKBACK flag to release XT1 for operation
	}
}

void enable_XT2 (void)
{
	P7SEL |= BIT2; 												// Choose port 7.2 for XT2 operation, page 100 DS, page 164 UG
	UCSCTL6 &= ~XT2OFF; 										// Enable XT2CLK
	clear_XT2_OFFG();											// Allow XT2 to settle
	UCSCTL6 &= ~XT2DRIVE_3;										// Set the lowest drive setting for 4 MHz crystal
}

void clear_XT2_OFFG (void)
{
    do
    {
    	UCSCTL7 &= ~XT2OFFG; 									// Clear XT2 fault flag
    	SFRIFG1 &= ~OFIFG; 										// Clear oscillator fault flag, see page 82 UG
    }
    while (SFRIFG1&OFIFG);										// Loop until OFIFG is cleared
}


uint8_t clear_XT2_OFFG_timeout (uint32_t timeout)
{
    do {
        UCSCTL7 &= ~XT2OFFG;                        			// Clear XT2 fault flag
        SFRIFG1 &= ~OFIFG;										// Clear oscillator fault flag, see page 82 UG
    } while ((SFRIFG1&OFIFG) && --timeout);						// Do until timeout has expired or flag has been cleared

    if (timeout)
        return (UCS_STATUS_OK);									// Timeout has not expired
    else
        return (UCS_STATUS_ERROR);								// Timeout has expired
}


void disable_XT2 (void)
{
    UCSCTL6 |= XT2OFF;                                          // Disable XT2 oscillator
}

