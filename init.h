/*
 * init.h
 *
 *  Created on: Nov 30, 2016
 *      Author: kdolgikh
 */

/** \file init.h
 * 	\brief Contains initialization function prototypes
 */

#ifndef INIT_H_
#define INIT_H_


/**
 * 	\brief Enables fast wake up time from LPM0-4
 *
 * 	Currently, disables SVSL and SVML, but may be changed later.
 */
//void cpu_fast_wakeup (void);


/** \brief Perform clocks setup
 *
 * 	Execution flow:
 * 	1. Set ports to output ACLK and SMCLK.
 * 	2. Select DCO frequency range to match specific MCLK frequency
 * 	3. Set FLL divider and multiplier values to produce specific MCLK frequency
 * 	4. Choose sources for ACLK, SMCLK and MCLK.
 * 	5. Clear oscillator fault flags. See NOTE.
 * 	6. Enable oscillator fault interrupts for the event log.
 *
 *	NOTE. According to MSP430F5659 Errata UCS12, changing the SELM/SELS/SELA bits
 *	in the UCSCTL4 register may set XT1/XT2 fault flags, which are needed to be
 *	cleared by user software.
 *
 */
void clocks_init (void);

//void TA0_init (void);

/**
 * 	\brief Initialize timer A1 for sleep_timer function
 */
//void TA1_init (void);


void TA2_init (void);

/**
 *
 */
void ports_init (void);


/**
 *  \brief Function to set up port 1.3 as a user button
 */
//void user_button_init (void);


/** \brief Perform device initialization
 *
 *  Execution flow:
 *  1. Disable watchdog timer.
 *  2. Enable XT1.
 *  3. Set up VCORE level to 3 to support 20 MHz MCLK.
 *
 */
void device_init (void);

void Radio_Init(void);

void disable_SVSM(void);

#endif /* INIT_H_ */
