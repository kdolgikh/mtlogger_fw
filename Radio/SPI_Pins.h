/*
 * SPI_Pins.h
 *
 *  Created on: Oct 20, 2015
 *      Author: cgoss
 *
 */

#include "CC110l.h"
#include "driverlib.h"

#ifndef SPI_PINS_H_
#define SPI_PINS_H_

// SPI Pins on MSP430. Port 8
#define SOMI BIT3
#define SIMO BIT2
#define SPCLK BIT1
#define CS BIT0

// SPI Registers
#define Port_Reg_Sel P8SEL 				// Port function select 1
#define Port_Reg_Dir P8DIR				// Port direction select
#define Port_In P8IN					// Input values for SPI port
#define CS_Register P8OUT				// Chip select for the slave device
#define CS_RegisterDir P8DIR			// Direction register for chip select pin
#define USCI_Control_Reg0 UCA1CTL0 		// USCI control reg 0
#define USCI_Control_Reg1 UCA1CTL1		// USCI control reg 1
#define USCI_Modulation_Upper UCA1BR1	// Upper byte of modulation value
#define USCI_Modulation_Lower UCA1BR0	// Lower byte of modulation vale
#define USCI_Status_Reg UCA1STAT		// Status register
#define USCI_Interrupt_Flags UCA1IFG	// Interrupt flag register
#define USCI_TX_Reg UCA1TXBUF			// Transmit buffer
#define USCI_RX_Reg UCA1RXBUF			// Receive buffer

// Register masks
#define USCI_RX_Flag UCRXIFG			// Mask for RX complete
#define USCI_TX_Flag UCTXIFG			// Mask for TX complete

// GDO Ports on MSP430
#define MSP_RX_Port_DIR		P1DIR	// Direction register
#define MSP_RX_Port_OUT		P1OUT	// Out value register
#define MSP_RX_Port_REN		P1REN	// Resistor enable register
#define MSP_RX_Port_IN		P1IN	// In value register


#endif /* SPI_PINS_H_ */
