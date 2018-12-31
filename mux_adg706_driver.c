/*
 * mux_adg706_driver.c
 *
 *  Created on: Dec 2, 2016
 *      Author: kdolgikh
 */

/** \file mux_adg706_driver.c
 * 	\brief Analog Devices 16 channel mux ADG706B driver source file
 */

#ifndef MUX_ADG706_DRIVER_C_
#define MUX_ADG706_DRIVER_C_

#include "msp430.h"
#include "mux_adg706_driver.h"
#include "states.h"

void mux_adg706_ports_init (void)
{
	P4DIR |= A3+A2+A1+A0+MUX_EN;										// Set output direction
	P4OUT &= ~(A3+A2+A1+A0+MUX_EN);										// Set output LOW; MUX is disabled.
}

void disable_mux_adg706 (void)
{
	P4OUT &= ~MUX_EN;
}

void switch_mux_ch (uint8_t ch_num)
{
    switch(ch_num)
    {
        case(CHANNEL1):
            switch_mux_adg706_ch1();               // Switch to channel 1
            break;
        case(CHANNEL2):
            switch_mux_adg706_ch2();
            break;
        case(CHANNEL3):
            switch_mux_adg706_ch3();
            break;
        case(CHANNEL4):
            switch_mux_adg706_ch4();
            break;
        case(CHANNEL5):
            switch_mux_adg706_ch5();
            break;
        case(CHANNEL6):
            switch_mux_adg706_ch6();
            break;
        case(CHANNEL7):
            switch_mux_adg706_ch7();
            break;
        case(CHANNEL8):
            switch_mux_adg706_ch8();
            break;
        case(CHANNEL9):
            switch_mux_adg706_ch9();
            break;
        case(CHANNEL10):
            switch_mux_adg706_ch10();
            break;
        case(CHANNEL11):
            switch_mux_adg706_ch11();
            break;
        case(CHANNEL12):
            switch_mux_adg706_ch12();
            break;
        case(CHANNEL13):
            switch_mux_adg706_ch13();
            break;
        case(CHANNEL14):
            switch_mux_adg706_ch14();
            break;
        case(CHANNEL15):
            switch_mux_adg706_ch15();
            break;
        case(CHANNEL16):
            switch_mux_adg706_ch16();           // Switch to channel 16
            break;
    }
}

void switch_mux_adg706_ch1 (void)
{
	P4OUT &= ~(A3+A2+A1+A0);
	P4OUT |= MUX_EN;
}

void switch_mux_adg706_ch2 (void)
{
	P4OUT &= ~(A3+A2+A1);
	P4OUT |= A0+MUX_EN;
}

void switch_mux_adg706_ch3 (void)
{
	P4OUT &= ~(A3+A2+A0);
	P4OUT |= A1+MUX_EN;
}

void switch_mux_adg706_ch4 (void)
{
	P4OUT &= ~(A3+A2);
	P4OUT |= A1+A0+MUX_EN;
}

void switch_mux_adg706_ch5 (void)
{
	P4OUT &= ~(A3+A1+A0);
	P4OUT |= A2+MUX_EN;
}

void switch_mux_adg706_ch6 (void)
{
	P4OUT &= ~(A3+A1);
	P4OUT |= A2+A0+MUX_EN;
}

void switch_mux_adg706_ch7 (void)
{
	P4OUT &= ~(A3+A0);
	P4OUT |= A2+A1+MUX_EN;
}

void switch_mux_adg706_ch8 (void)
{
	P4OUT &= ~A3;
	P4OUT |= A2+A1+A0+MUX_EN;
}

void switch_mux_adg706_ch9 (void)
{
	P4OUT &= ~(A2+A1+A0);
	P4OUT |= A3+MUX_EN;
}


void switch_mux_adg706_ch10 (void)
{
	P4OUT &= ~(A2+A1);
	P4OUT |= A3+A0+MUX_EN;
}

void switch_mux_adg706_ch11 (void)
{
	P4OUT &= ~(A2+A0);
	P4OUT |= A3+A1+MUX_EN;
}

void switch_mux_adg706_ch12 (void)
{
	P4OUT &= ~A2;
	P4OUT |= A3+A1+A0+MUX_EN;
}

void switch_mux_adg706_ch13 (void)
{
	P4OUT &= ~(A1+A0);
	P4OUT |= A3+A2+MUX_EN;
}

void switch_mux_adg706_ch14 (void)
{
	P4OUT &= ~A1;
	P4OUT |= A3+A2+A0+MUX_EN;
}

void switch_mux_adg706_ch15 (void)
{
	P4OUT &= ~A0;
	P4OUT |= A3+A2+A1+MUX_EN;
}

void switch_mux_adg706_ch16 (void)
{
	P4OUT |= A3+A2+A1+A0+MUX_EN;
}

#endif /* MUX_ADG706_DRIVER_C_ */
