/*
 * init.c
 *
 *  Created on: Nov 30, 2016
 *      Author: kdolgikh
 */

/** \file init.c
 * 	\brief Contains initialization functions
 */

#include "driverlib.h"
#include "init.h"
#include "adc_driver.h"
#include "mux_adg706_driver.h"
#include "ucs_functions.h"
#include "hal_SPI.h"
#include "sdcard.h"
#include "Radio/CC110l.h"
#include "Radio/SPI_Library.h"
#include "sleep_timer.h"

void clocks_init (void)
{
	P1SEL |= BIT0;													// Set pins to output ACLK
	P1DIR |= BIT0; 													// ACLK output at port 1.0

	P3SEL |= BIT4;													// Set pins to output SMCLK
	P3DIR |= BIT4; 													// SMCLK output at port 3.4

	UCSCTL1 = DCORSEL_5; 											// For MCLK of 20 MHz
    UCSCTL2 = FLLD_1 + FLLN0 + FLLN1 + FLLN2 + FLLN3 + FLLN5 + FLLN8;// Set FLLD divide by 2 and FLLN to 303 to provide 20 MHz MCLK
    UCSCTL4 = SELS__DCOCLK + SELM__DCOCLK; 							// Set sources to DCOCLK; ACLK is sourced from XT1 by default
//    UCSCTL8 &= ~SMCLKREQEN;											// Disable conditional requests for SMCLK
//    UCSCTL8 &= ~MCLKREQEN;											// Disable conditional request for MCLK

    BAKCTL |= BAKDIS;												// Disable battery backup system

    enable_XT1_RTC();                                               // Enable XT1 crystal and RTC

//    UCSCTL7 &= ~(XT1LFOFFG + DCOFFG);								// Clear XT1 and DCO fault flags
//    SFRIFG1 &= ~(OFIFG);											// Clear oscillator fault flag, see page 82 UG
}

/*void cpu_fast_wakeup (void)
{
	PMMCTL0_H = PMMPW_H;											// Enter password to change PMM registers
	SVSMLCTL &= ~(SVSLE + SVMLE);									// Disable SVSL and SVSM for the fastest wake up time // May be temporarily
	PMMCTL0_H = 0x00;												// Clear PMM password
}*/

/*void TA1_init (void)
{
	TA1CTL |= TASSEL_1; 											// TA clock source is ACLK
	TA1CTL |= ID_3;													// Divide ACLK by 8
	TA1EX0 |= TAIDEX_3;												// Additionally divide by 4. Overflow is every 64 seconds
	TA1CTL |= TACLR;												// Clear TA1R, see p 465 UG
	TA1CTL |= TAIE;													// Enable TAIFG interrupt
	TA1CTL &= ~TAIFG;												// Clear TAIFG
}*/

void TA2_init (void)
{
	TA2CTL |= TASSEL_1; 											// TA clock source is ACLK
	TA2CTL |= ID_3;													// Divide ACLK by 8 . Overflow is 16 seconds
	TA2CTL |= TACLR;												// Clear TA1R, see p 465 UG
	TA2CTL |= TAIE;													// Enable TAIFG interrupt
	TA2CTL &= ~TAIFG;												// Clear TAIFG
	TA2CCTL1 |= CCIE;												// Enable CCIFG interrupt
	TA2CCTL1 &= ~CCIFG;												// Clear CCIFG
}

/*void TA0_init (void)                                                // Used to measure duration of certain operations
{
    TA0CTL |= TASSEL_2;                                             // TA clock source is MCLK
    TA0CTL |= TAIE;                                                 // Enable TAIFG interrupt
    TA0CTL &= ~TAIFG;                                               // Clear TAIFG
}*/

// All unused ports output direction, low to reduce power consumption
void ports_init (void)
{
	P4OUT &= ~BIT6;													// Power MUX pin Low -> power from the battery
	P4DIR |= BIT6; 													// Power MUX pin - output direction

	P4OUT &= ~BIT7;													// SD-card MUX ON pin Low -> power not applied to SD-card
	P4DIR |= BIT7;													// SD-card power MUX ON pin - output direction
	// Since it is the output direction and POUT is undefined upon reset,
	// POUT was High, thus VCC was applied to SD-card!

	// Unused pins port 1 (and RF GDO0, GDO2)
	P1DIR |= BIT1|BIT2|BIT4|BIT6|BIT7;								// Output direction
	P1OUT &= ~(BIT1|BIT2|BIT4|BIT6|BIT7);							// Set low

	// Unused pins port 2 - all pins are unused
	P2DIR = 0xFF;													// Set all pins as output
	P2OUT = 0x00;													// Set all pins low

	// Unused pins port 3
	P3DIR |= BIT0|BIT1|BIT2|BIT3|BIT5|BIT6|BIT7;					// Output direction
	P3OUT &= ~(BIT0|BIT1|BIT2|BIT3|BIT5|BIT6|BIT7);					// Set low

	// Unused pins port 5
	P5DIR |= BIT0|BIT1|BIT4|BIT5|BIT6|BIT7;
	P5OUT &= ~(BIT0|BIT1|BIT4|BIT5|BIT6|BIT7);

	// Unused pins port 6
	P6DIR = 0xFF;													// Set all pins as output
	P6OUT = 0x00;													// Set all pins low

	// Unused pins port 7
	P7DIR |= BIT4|BIT5|BIT6|BIT7;
	P7OUT &= ~(BIT4|BIT5|BIT6|BIT7);

	// Unused pins port 9
	P9DIR |= BIT4|BIT5|BIT6|BIT7;
	P9OUT &= ~(BIT4|BIT5|BIT6|BIT7);
}

void user_button_init(void)
{
    P1DIR &= ~BIT3;                                                     //input direction //should be 0 by default, page 426 UG
    P1OUT |= BIT3;                                                      //pull-up is configured
    P1REN |= BIT3;                                                      //pull-up is enabled
    P1IES |= BIT3;                                                      //high-to-low transition
    //writing to P1OUT, P1DIR, P1REN (page 412 UG), P1IES (page 413 UG) can result in setting P1IFG
    //Hence, we'll clear P1IFG before enabling port interrupt with P1IE
    P1IFG &= ~BIT3;                                                     // set P1IFG to 0
    P1IE |= BIT3;                                                   //enable interrupt
}


void Radio_Init(void)
{
	SPI_Strobe(SRES, Get_RX_FIFO); // Reset radio

	Sleep_Timer_Fast(5);

	// Rf settings for CC110L, generated by SmartRF Studio
	// This is the very bottom option on the dropdown list
	// 250 kBaud, Dev.: 127 kHz, Mod.: GFSK, RX BW: 540 kHz
	SPI_Send(IOCFG0,0x2E);		 //Set GDO_RX to tri-state
	SPI_Send(IOCFG1,0x2E);
	SPI_Send(IOCFG2,0x2E);
//	SPI_Send(PKTCTRL0,0x05);     //Packet Automation Control
//	SPI_Send(PKTCTRL1,BIT2 | BIT3);	// Append RSSI and CRC and enable autoflush of bad pkts
//	SPI_Send(FSCTRL1,0x12);      //Frequency Synthesizer Control
//	SPI_Send(FREQ2,0x21);        //Frequency Control Word, High Byte
//	SPI_Send(FREQ1,0x62);        //Frequency Control Word, Middle Byte
//	SPI_Send(FREQ0,0x76);        //Frequency Control Word, Low Byte
//	SPI_Send(MDMCFG4,0x2D);      //Modem Configuration
//	SPI_Send(MDMCFG3,0x3B);      //Modem Configuration
//	SPI_Send(MDMCFG2,0x93);      //Modem Configuration
//	SPI_Send(DEVIATN,0x62);      //Modem Deviation Setting
//	SPI_Send(MCSM0,0b00101000);        //Main Radio Control State Machine Configuration
//	SPI_Send(FOCCFG,0x1D);       //Frequency Offset Compensation Configuration
//	SPI_Send(BSCFG,0x1C);        //Bit Synchronization Configuration
//	SPI_Send(AGCCTRL2,0xC7);     //AGC Control
//	SPI_Send(AGCCTRL1,0x00);     //AGC Control
//	SPI_Send(AGCCTRL0,0xB0);     //AGC Control
//	SPI_Send(0x20,0xFB);		 //Use setting from SmartRF Studio
//	SPI_Send(FREND1,0xB6);       //Front End RX Configuration
//	SPI_Send(FSCAL3,0xEA);       //Frequency Synthesizer Calibration
//	SPI_Send(FSCAL2,0x2A);       //Frequency Synthesizer Calibration
//	SPI_Send(FSCAL1,0x00);       //Frequency Synthesizer Calibration
//	SPI_Send(FSCAL0,0x1F);       //Frequency Synthesizer Calibration
//	SPI_Send(TEST0,0x09);        //Various Test Settings
	SPI_Strobe(SPWD, Get_RX_FIFO);	// Send radio to sleep
}

void disable_SVSM (void)
{
	PMMCTL0_H = PMMPW_H;												// Enter password to change PMM registers
	PMMRIE &= ~(SVSHPE | SVSLPE);									// Disable SVS low and high side interrupts
	SVSMHCTL &= ~(SVMHE | SVSHE);									// Disable high side SVS-SVM
	SVSMLCTL &= ~(SVMLE | SVSLE);									// Disable low side SVS-SVM
	PMMCTL0_H = 0;													// Clear PMM password
}

void device_init(void)
{
	WDTCTL = WDTPW | WDTHOLD;                                        // Stop watchdog timer
    PMM_setVCore(PMM_CORE_LEVEL_3);                                 // Set VCore level to 3 to support 20 MHz MCLK
    clocks_init();                                                  // Set up clocks
    ports_init();                                                   // Initialize ports including unused ports
//    TA0_init();
//    TA1_init();                                                     // Set up TA1 for sleep_timer_slow
    TA2_init();                                                     // Set up TA2 for sleep_timer_fast
    adc_spi_init();                                                 // Init ADC pins and SPI
    SPI_Init();														// Init radio SPI
    mux_adg706_ports_init();                                        // Init MUX ADG706 pins
    sdc_CS_Init();                                                  // Initialize uSD-card CS pin
    user_button_init();                                             // P1.3 init
    disable_SVSM();													// Disable SVS, SVM high and low sides
}
