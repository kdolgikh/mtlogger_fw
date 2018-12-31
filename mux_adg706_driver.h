/*
 * mux_adg706_driver.h
 *
 *  Created on: Dec 2, 2016
 *      Author: kdolgikh
 */

/** \file mux_adg706_driver.h
 * 	\brief Analog Devices 16 channel mux ADG706B driver header file
 */

#ifndef MUX_ADG706_DRIVER_H_
#define MUX_ADG706_DRIVER_H_

#include "msp430.h"
#include "stdint.h"
#include "states.h"

#define A0 BIT0
#define A1 BIT1
#define A2 BIT2
#define A3 BIT3
#define MUX_EN BIT4

#define ALL_CHANNELS    0
#define CHANNEL1        1
#define CHANNEL2        2
#define CHANNEL3        3
#define CHANNEL4        4
#define CHANNEL5        5
#define CHANNEL6        6
#define CHANNEL7        7
#define CHANNEL8        8
#define CHANNEL9        9
#define CHANNEL10       10
#define CHANNEL11       11
#define CHANNEL12       12
#define CHANNEL13       13
#define CHANNEL14       14
#define CHANNEL15       15
#define CHANNEL16       16

/**	\brief Set up MUX ADG706 ports
 *
 * 	Ports connections:
 *  MCU		MUX
 * 	P4.0	A0
 * 	P4.1	A1
 * 	P4.2	A2
 * 	P4.3	A3
 * 	P4.4	EN
 */
void mux_adg706_ports_init (void);


/** \brief Disable MUX ADG706
 *
 */
void disable_mux_adg706 (void);


/**
 * 	\brief Switch MUX ADG706 to the specific channel_number
 */
void switch_mux_ch (uint8_t channel_number);


/** \brief Switch MUX ADG706 channels
 *
 */
void switch_mux_adg706_ch1 (void);
void switch_mux_adg706_ch2 (void);
void switch_mux_adg706_ch3 (void);
void switch_mux_adg706_ch4 (void);
void switch_mux_adg706_ch5 (void);
void switch_mux_adg706_ch6 (void);
void switch_mux_adg706_ch7 (void);
void switch_mux_adg706_ch8 (void);
void switch_mux_adg706_ch9 (void);
void switch_mux_adg706_ch10 (void);
void switch_mux_adg706_ch11 (void);
void switch_mux_adg706_ch12 (void);
void switch_mux_adg706_ch13 (void);
void switch_mux_adg706_ch14 (void);
void switch_mux_adg706_ch15 (void);
void switch_mux_adg706_ch16 (void);

#endif /* MUX_ADG706_DRIVER_H_ */
