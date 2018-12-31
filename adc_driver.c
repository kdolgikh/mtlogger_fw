/*
 * adc_driver.c
 *
 *  Created on: Dec 2, 2016
 *      Author: kdolgikh
 */


/**
 *	\file adc_driver.c
 * 	\brief TI ADS1247 driver source file
 */

#include "driverlib.h"
#include "adc_driver.h"
#include "states.h"
#include "stdint.h"
#include "crc8.h"
#include "USB_API/USB_Common/usb.h"
#include "send_over_usb.h"
#include "sleep_timer.h"
#include "data_prep.h"
#include "mux_adg706_driver.h"
#include "measure.h"


uint8_t adc_data_rate = ADC_DR_10SPS;
uint8_t adc_mux1_vrefcon = MUX1_VREFCON_VREF_ALT;

//ADC commands to read a single register
uint8_t adc_rreg_mux0[SINGLE_REG_RW_CMD_LGTH]  = {ADC_RREG_MUX0_BYTE1, ADC_BYTE2_SINGLE_REG, ADC_NOP};
uint8_t adc_rreg_vbias[SINGLE_REG_RW_CMD_LGTH] = {ADC_RREG_VBIAS_BYTE1, ADC_BYTE2_SINGLE_REG, ADC_NOP};
uint8_t adc_rreg_mux1[SINGLE_REG_RW_CMD_LGTH]  = {ADC_RREG_MUX1_BYTE1, ADC_BYTE2_SINGLE_REG, ADC_NOP};
uint8_t adc_rreg_sys0[SINGLE_REG_RW_CMD_LGTH]  = {ADC_RREG_SYS0_BYTE1, ADC_BYTE2_SINGLE_REG, ADC_NOP};
uint8_t adc_rreg_idac0[SINGLE_REG_RW_CMD_LGTH] = {ADC_RREG_IDAC0_BYTE1, ADC_BYTE2_SINGLE_REG, ADC_NOP};
uint8_t adc_rreg_idac1[SINGLE_REG_RW_CMD_LGTH] = {ADC_RREG_IDAC1_BYTE1, ADC_BYTE2_SINGLE_REG, ADC_NOP};
uint8_t adc_rreg_gpiocfg[SINGLE_REG_RW_CMD_LGTH] = {ADC_RREG_GPIOCFG_BYTE1, ADC_BYTE2_SINGLE_REG, ADC_NOP};
uint8_t adc_rreg_gpiodir[SINGLE_REG_RW_CMD_LGTH]	= {ADC_RREG_GPIODIR_BYTE1, ADC_BYTE2_SINGLE_REG, ADC_NOP};
uint8_t adc_rreg_gpiodat[SINGLE_REG_RW_CMD_LGTH]= {ADC_RREG_GPIODAT_BYTE1, ADC_BYTE2_SINGLE_REG, ADC_NOP};

//ADC command to reed the first four registers
uint8_t adc_rreg_first4[FOUR_REG_RW_CMD_LGTH] = {ADC_RREG_MUX0_BYTE1, ADC_BYTE2_FOUR_REG, ADC_NOP, ADC_NOP, ADC_NOP, ADC_NOP};

// ADC commands to read calibration registers
uint8_t adc_rreg_ofc[THREE_REG_RW_CMD_LGTH] = {ADC_RREG_OFC0_BYTE1, ADC_BYTE2_THREE_REG, ADC_NOP, ADC_NOP, ADC_NOP};
uint8_t adc_rreg_fsc[THREE_REG_RW_CMD_LGTH] = {ADC_RREG_FSC0_BYTE1, ADC_BYTE2_THREE_REG, ADC_NOP, ADC_NOP, ADC_NOP};

// ADC command to read all registers
const uint8_t adc_rreg_all[ALL_REG_R_CMD_LGTH]	=
{
								ADC_RREG_MUX0_BYTE1, ADC_BYTE2_ALL_REG,
								ADC_NOP, ADC_NOP, ADC_NOP, ADC_NOP, ADC_NOP,
								ADC_NOP, ADC_NOP, ADC_NOP, ADC_NOP, ADC_NOP,
								ADC_NOP, ADC_NOP, ADC_NOP, ADC_NOP, ADC_NOP
};

const struct																		// The struct with default values of ADC registers
{
	uint8_t adc_mux0;
	uint8_t adc_vbias;
	uint8_t adc_mux1;
	uint8_t adc_sys0;
	uint8_t adc_ofc0;
	uint8_t adc_ofc1;
	uint8_t adc_ofc2;
	uint8_t adc_fsc0;
	uint8_t adc_fsc1;
	uint8_t adc_fsc2;
	uint8_t adc_idac0;
	uint8_t adc_idac1;
	uint8_t adc_gpiocfg;
	uint8_t adc_gpiodir;
	uint8_t adc_gpiodat;
} adc_regs_default = {
	0x01,	// mux0
	0x00,	// vbias
	0x00,	// mux1
	0x00,	// sys0
	0x00,	// ofc0
	0x00,	// ofc1
	0x00,	// ofc2
	0x00,	// fsc0
	0x10,	// fsc1
	0x40,	// fsc2
	0x90,	// idac0
	0xFF,	// idac1
	0x00,	// gpiocfg
	0x00,	// gpiodir
	0x00	// gpiodat
};

uint8_t adc_calibration_register[CAL_REG_LGTH] = {0};								// Storage for a calibration register
uint8_t adc_regs_four[FOUR_REG_LGTH] = {0};											// Storage for the first four registers mux0,vbias, mux1, sys0
uint8_t adc_regs_all[ADC_NUM_REGISTERS] = {0};										// Storage for all ADC registers


uint8_t sysocal_enable = FALSE;


uint8_t adc_wreg_single[SINGLE_REG_RW_CMD_LGTH] = {0};													// Array for any WREG command
const uint8_t adc_wreg_sysgcal	= ADC_SYSGCAL;										// Write system gain calibration command
const uint8_t adc_wreg_selfocal = ADC_SELFOCAL;										// Write self offset calibration command
const uint8_t adc_wreg_sysocal = ADC_SYSOCAL;										// Write system offset calibration command


uint8_t adc_conv_result [CONV_LENGTH] = {0};										// ADC conversion result of 24 bits


adc_mux0_register *adc_mux0_ptr;													// Ptr to struct of type adc_mux0_reg
adc_mux1_register *adc_mux1_ptr;													// Ptr to struct of type adc_mux1_reg
adc_sys0_register *adc_sys0_ptr;													// Ptr to struct of type adc_sys0_reg


uint8_t adcWorkInProgress = FALSE;													// Flag that prevents other processes to affect ADC operation

void adc_spi_pins_init (void)
{
	P5DIR |= ADC_CS_N + ADC_RST_N;													// Set P5.2 (ADC_CS*) and P5.3 (ADC_RST*) output direction
	P5OUT |= ADC_CS_N + ADC_RST_N;													// Set P5.2 (ADC_CS*) P5.3 (ADC_RST*) HIGH
	P9DIR |= ADC_START;																// Set P9.0 (ADC_START) output direction
	P9OUT &= ~ADC_START;															// Set P9.0 (ADC_START) LOW; PXOUT is not initialized upon reset

	P9SEL |= BIT1 + BIT2 + BIT3;													// Select USCI function for ports P9.1, 9.2, 9.3

	P1OUT |= BIT5;																	// P1.5 (ADC_DRDY*) is initialized as input direction after reset. Set a pull-up resistor for P1.5
	P1REN |= BIT5;																	// Pull-up resistor is enabled
	P1IES |= BIT5;																	// Select interrupt on a falling edge for P1.5 (ADC_DRDY*)

	//writing to P1OUT, P1DIR, P1REN (page 412 UG), P1IES (page 413 UG) can result in setting P1IFG
    //Hence, we'll clear P1IFG before enabling port interrupt with P1IE
	P1IFG &= ~BIT5;																	// Clear P1.5 IFG
	P1IE |= BIT5;																	// Enable interrupts for P1.5 (ADC_DRDY*)
}


void adc_spi_init (void)
{
	//Sleep_Timer_Fast(66);															// Wait for 16 ms before configuring ADC
	UCA2CTL1 |= UCSWRST;															// Hold USCI in reset. Set by default
	//When set, the UCSWRST bit resets the UCRXIE, UCTXIE,
	//UCRXIFG, UCOE, and UCFE bits, and sets the UCTXIFG flag; page 972 UG

	UCA2CTL0 |= UCMSB;																// MSB first to interface with ADS1247
	UCA2CTL0 |= UCMST;																// Master mode
	UCA2CTL0 |= UCSYNC;																// Synchronous mode (SPI)
	//After reset, UCA2CTL0 default values are: 8 bit length, 3-pin SPI
	//ADS1247 uses Mode0 (polarity and phase are 0, as by default)

	UCA2CTL1 |= UCSSEL__SMCLK;														// SMCLK of 20 MHz is a source for SPI clock
	UCA2BR0 = 0x0A;																	// Divide SMCLK by 10 to get 2 MHz SCLK. Max ADS1247 SCLK is 2.05 MHz, page 10 ADS1247 UG
	UCA2BR1 = 0;																	// Set to 0 to use only UCA2BR0 value

	adc_spi_pins_init();															// Initialize ADC SPI pins

	UCA2CTL1 &= ~UCSWRST;															// Release USCI for operation
}


device_state adc_reset (void)
{
	P5OUT &= ~ADC_RST_N;
    __delay_cycles(40);																// Duration of the RST pulse
    P5OUT |= ADC_RST_N;

    adcWorkInProgress = TRUE;
    Sleep_Timer_Fast(3);															// Settling delay of 0.7 ms (should be greater than 0.6 ms)
    adcWorkInProgress = FALSE;

    if(!USBconnected)
    	return adc_set;
    else
    	return usb_connected;
}


uint8_t adc_spi_rreg_single (uint8_t *adc_rreg_cmd)
{
	uint8_t i;
	uint8_t adc_register = 0;

	P5OUT &= ~ADC_CS_N;																// Set ADC_CS* LOW to enable SPI communication with ADC

	for (i = 0; i <= COMMAND_BYTE_3; i++)
	{
		while (!(UCA2IFG & UCTXIFG));												// Wait until TX is done
		UCA2TXBUF = *(adc_rreg_cmd + i);											// Send command bytes
		while (!(UCA2IFG & UCRXIFG));												// Wait until RX is done
		adc_register = UCA2RXBUF;													// Only the third received byte is required
	}

	__delay_cycles(30);																// Make sure Tsccs > 1.7 us

	P5OUT |= ADC_CS_N;																// Set ADC_CS* HIGH to disable SPI communication with ADC

	return adc_register;
}


void adc_spi_rreg_first4 (uint8_t *adc_rreg_cmd, uint8_t *four_reg_stg)
{
	uint8_t i;

	P5OUT &= ~ADC_CS_N;																// Set ADC_CS* LOW to enable SPI communication with ADC

	for (i = 0; i <= COMMAND_BYTE_6; i++)
	{
		while (!(UCA2IFG & UCTXIFG));												// Wait until TX is done
		UCA2TXBUF = *(adc_rreg_cmd + i);											// Send command bytes
		while (!(UCA2IFG & UCRXIFG));												// Wait until RX is done
		if (i < COMMAND_BYTE_3)
			*four_reg_stg = UCA2RXBUF;												// Discard the first two bytes
		else
		{
			*four_reg_stg = UCA2RXBUF;												// Store bytes 3-6 into the storage array
			four_reg_stg++;
		}
	}

	__delay_cycles(30);																// Make sure Tsccs > 1.7 us

	P5OUT |= ADC_CS_N;																// Set ADC_CS* HIGH to disable SPI communication with ADC
}


void adc_spi_rreg_cal_register (uint8_t *cal_reg_cmd, uint8_t *adc_cal_register)
{
	uint8_t i;

	P5OUT &= ~ADC_CS_N;																// Set ADC_CS* LOW to enable SPI communication with ADC

	for (i = 0; i <= COMMAND_BYTE_5; i++)
	{
		while (!(UCA2IFG & UCTXIFG));												// Wait until TX is done
		UCA2TXBUF = *(cal_reg_cmd + i);												// Send command bytes
		while (!(UCA2IFG & UCRXIFG));												// Wait until RX is done
		if (i < COMMAND_BYTE_3)
			*adc_cal_register = UCA2RXBUF;											// Discard the first two bytes
		else
		{
			*adc_cal_register = UCA2RXBUF;											// Store bytes 3-5 into the array for calibration registers
			adc_cal_register++;
		}
	}

	__delay_cycles(30);																// Make sure Tsccs > 1.7 us

	P5OUT |= ADC_CS_N;
}


void adc_spi_rreg_all (uint8_t *adc_regs_all)
{
	uint8_t i;

	P5OUT &= ~ADC_CS_N;																// Set ADC_CS* LOW to enable SPI communication with ADC

	for (i = 0; i <= COMMAND_BYTE_17; i++)
	{
		while (!(UCA2IFG & UCTXIFG));												// Wait until TX is done
		UCA2TXBUF = *(adc_rreg_all + i);											// Send command bytes
		while (!(UCA2IFG & UCRXIFG));												// wait until RX is done
		if (i < COMMAND_BYTE_3)
			*adc_regs_all = UCA2RXBUF;												// Discard the first two bytes
		else
		{
			*adc_regs_all = UCA2RXBUF;												// Store bytes 3-17 into the array for all registers
			adc_regs_all++;
		}
	}

	__delay_cycles(30);																// Make sure Tsccs > 1.7 us

	P5OUT |= ADC_CS_N;																// Set ADC_CS* HIGH to disable SPI communication with ADC
}


void adc_spi_define_wreg_mux0 (adc_mux0_register *adc_mux0_param)
{
	uint8_t mux0_register;

	mux0_register =																	// Define the contents of the register
			adc_mux0_param->adc_bcs +
			adc_mux0_param->adc_mux_pos_input +
			adc_mux0_param->adc_mux_neg_input;

	*adc_wreg_single = ADC_WREG_MUX0_BYTE1;											// Write starting from MUX0 register
	*(adc_wreg_single + COMMAND_BYTE_2) = ADC_BYTE2_SINGLE_REG;						// Write only one register at a time
	*(adc_wreg_single + COMMAND_BYTE_3) = mux0_register;							// Data to be written to the register
}


void adc_spi_define_wreg_mux1 (adc_mux1_register *adc_mux1_param)
{
	uint8_t mux1_register;

	mux1_register =																	// Define the contents of the register
			adc_mux1_param->adc_clkstat +
			adc_mux1_param->adc_vrefcon +
			adc_mux1_param->adc_refselt +
			adc_mux1_param->adc_muxcal;

	*adc_wreg_single = ADC_WREG_MUX1_BYTE1;											// Write starting from MUX1 register
	*(adc_wreg_single + COMMAND_BYTE_2) = ADC_BYTE2_SINGLE_REG;						// Write only one register at a time
	*(adc_wreg_single + COMMAND_BYTE_3) = mux1_register;							// Data to be written to the register
}


void adc_spi_define_wreg_sys0 (adc_sys0_register *adc_sys0_param)
{
	uint8_t sys0_register;

	sys0_register =																	// Define the contents of the register
			adc_sys0_param->adc_pga_gain +
			adc_sys0_param->adc_data_rate;

	*adc_wreg_single = ADC_WREG_SYS0_BYTE1;											// Write starting from SYS0 register
	*(adc_wreg_single + COMMAND_BYTE_2) = ADC_BYTE2_SINGLE_REG;						// Write only one register at a time
	*(adc_wreg_single + COMMAND_BYTE_3) = sys0_register;							// Data to be written to the register
}

// Need to make this function generic, applicable to any register and not just three
uint8_t adc_spi_wreg_single (void)
{
	uint8_t i;
	uint8_t adc_reg = 0;

	P5OUT &= ~ADC_CS_N;																// Set ADC_CS* LOW to enable SPI communication with ADC

	for (i = 0; i <= COMMAND_BYTE_3; i++)
	{
		while (!(UCA2IFG & UCTXIFG));												// Wait until TX is done
		UCA2TXBUF = *(adc_wreg_single + i);											// Send command bytes
		while (!(UCA2IFG & UCRXIFG));												// Wait until RX is done
		adc_reg = UCA2RXBUF;														// Read from RX buffer to clear the RXIFG
	}

	__delay_cycles(30);																// Make sure Tsccs > 1.7 us

	if (*adc_wreg_single == ADC_WREG_MUX0_BYTE1)									// Does the first element equal to MUX0 cmd?
		adc_reg = adc_spi_rreg_single (adc_rreg_mux0);								// If yes, read MUX0 register
	else if (*adc_wreg_single == ADC_WREG_MUX1_BYTE1)								// Does the first element equal to MUX1 cmd?
		adc_reg = adc_spi_rreg_single (adc_rreg_mux1);								// If yes, read MUX1 register
	else if (*adc_wreg_single == ADC_WREG_SYS0_BYTE1)								// Does the first element equal to SYS0 cmd?
		adc_reg = adc_spi_rreg_single (adc_rreg_sys0);								// If yes, read SYS0 register

	if (adc_reg == *(adc_wreg_single + COMMAND_BYTE_3))								// Check if read value equals written value
		return WRITE_SUCCESS;
	else
		return WRITE_FAIL;
}


uint8_t adc_spi_wreg_mux0 (void)
{
	adc_mux0_register adc_mux_0 =
	{
		adc_mux_0.adc_bcs = MUX0_BCS_OFF,
		adc_mux_0.adc_mux_pos_input = SP_AIN2,		// invert the inputs from the default
		adc_mux_0.adc_mux_neg_input = SN_AIN3
	};

	adc_mux0_ptr = &adc_mux_0;

	adc_spi_define_wreg_mux0(adc_mux0_ptr);

	return adc_spi_wreg_single();
}


uint8_t adc_spi_wreg_mux1_sys0 (adc_mux1_register *adc_mux1_param, adc_sys0_register *adc_sys0_param)
{
	uint8_t adc_w_cmd[TWO_REG_RW_CMD_LGTH] = {0};														// write command
	uint8_t mux1_register;
	uint8_t sys0_register;
	uint8_t i;
	uint8_t adc_reg = 0;

	mux1_register =																	// Define the contents of the register
			adc_mux1_param->adc_clkstat +
			adc_mux1_param->adc_vrefcon +
			adc_mux1_param->adc_refselt +
			adc_mux1_param->adc_muxcal;

	sys0_register =																	// Define the contents of the register
				adc_sys0_param->adc_pga_gain +
				adc_sys0_param->adc_data_rate;

	*adc_w_cmd = ADC_WREG_MUX1_BYTE1;												// Write starting from MUX1 register
	*(adc_w_cmd + COMMAND_BYTE_2) = ADC_BYTE2_TWO_REG;								// Write two registers at once
	*(adc_w_cmd + COMMAND_BYTE_3) = mux1_register;									// Data to be written to the register
	*(adc_w_cmd + COMMAND_BYTE_4) = sys0_register;									// Data to be written to the register

	P5OUT &= ~ADC_CS_N;																// Set ADC_CS* LOW to enable SPI communication with ADC

	for (i = 0; i <= COMMAND_BYTE_4; i++)
	{
		while (!(UCA2IFG & UCTXIFG));												// Wait until TX is done
		UCA2TXBUF = *(adc_w_cmd + i);												// Send command bytes
		while (!(UCA2IFG & UCRXIFG));												// Wait until RX is done
		adc_reg = UCA2RXBUF;														// Read from RX buffer to clear the RXIFG
	}
	__delay_cycles(30);																// Make sure Tsccs > 1.7 us

	P5OUT |= ADC_CS_N;

	__delay_cycles(20);																// Make sure Tcspw > 1.22 us

	adc_reg = adc_spi_rreg_single (adc_rreg_mux1);
	if (adc_reg == mux1_register)
	{
		__delay_cycles(20);															// Make sure Tcspw > 1.22 us
		adc_reg = adc_spi_rreg_single (adc_rreg_sys0);
		if (adc_reg == sys0_register)
			return WRITE_SUCCESS;
		else
			return WRITE_FAIL;
	}
	else
		return WRITE_FAIL;
}


uint8_t adc_spi_wreg_first4 (adc_mux0_register *adc_mux0_param,
										adc_mux1_register *adc_mux1_param,
										adc_sys0_register *adc_sys0_param)
{
	uint8_t adc_w_cmd[FOUR_REG_RW_CMD_LGTH] = {0};									// Write command
	uint8_t mux0_register;
	uint8_t vbias_register = 0;														// vbias register is 0
	uint8_t mux1_register;
	uint8_t sys0_register;
	uint8_t i;
	uint8_t adc_reg = 0;

	mux0_register =																	// Define the contents of the register
			adc_mux0_param->adc_bcs +
			adc_mux0_param->adc_mux_pos_input +
			adc_mux0_param->adc_mux_neg_input;

	mux1_register =																	// Define the contents of the register
			adc_mux1_param->adc_clkstat +
			adc_mux1_param->adc_vrefcon +
			adc_mux1_param->adc_refselt +
			adc_mux1_param->adc_muxcal;

	sys0_register =																	// Define the contents of the register
				adc_sys0_param->adc_pga_gain +
				adc_sys0_param->adc_data_rate;

	*adc_w_cmd = ADC_WREG_MUX0_BYTE1;												// Write starting from MUX0 register
	*(adc_w_cmd + COMMAND_BYTE_2) = ADC_BYTE2_FOUR_REG;								// Write four registers at once
	*(adc_w_cmd + COMMAND_BYTE_3) = mux0_register;									// Data to be written to the register
	*(adc_w_cmd + COMMAND_BYTE_4) = vbias_register;									// Data to be written to the register
	*(adc_w_cmd + COMMAND_BYTE_5) = mux1_register;									// Data to be written to the register
	*(adc_w_cmd + COMMAND_BYTE_6) = sys0_register;									// Data to be written to the register

	P5OUT &= ~ADC_CS_N;																// Set ADC_CS* LOW to enable SPI communication with ADC

	for (i = 0; i <= COMMAND_BYTE_6; i++)
	{
		while (!(UCA2IFG & UCTXIFG));												// Wait until TX is done
		UCA2TXBUF = *(adc_w_cmd + i);												// Send command bytes
		while (!(UCA2IFG & UCRXIFG));												// Wait until RX is done
		adc_reg = UCA2RXBUF;														// Read from RX buffer to clear the RXIFG
	}
	__delay_cycles(30);																// Make sure Tsccs > 1.7 us

	P5OUT |= ADC_CS_N;

	__delay_cycles(20);																// Make sure Tcspw > 1.22 us

	adc_spi_rreg_first4(adc_rreg_first4, adc_regs_four);							// Read the first four registers

	if ((*adc_regs_four == mux0_register) &&
			 (*(adc_regs_four + 1) == vbias_register) &&
			 (*(adc_regs_four + 2) == mux1_register) &&
			 (*(adc_regs_four + 3) == sys0_register))
		return WRITE_SUCCESS;
	else
		return WRITE_FAIL;
}


device_state adc_setup (uint8_t data_rate, uint8_t mux1_vrefcon)
{
	uint8_t status = 0;
	uint8_t num_write = 0;

	adc_mux0_register adc_mux_0 =
	{
		adc_mux_0.adc_bcs = MUX0_BCS_OFF,
		adc_mux_0.adc_mux_pos_input = SP_AIN2, //SP_AIN2,										// Invert the inputs from the default
		adc_mux_0.adc_mux_neg_input = SN_AIN3//SN_AIN3
	};

	adc_mux1_register adc_mux_1 =
	{
		adc_mux_1.adc_clkstat = MUX1_CLKSTAT_INT_OSC,								// Internal oscillator
		adc_mux_1.adc_vrefcon = mux1_vrefcon,										// Control the VREF module
		adc_mux_1.adc_refselt = MUX1_REFSELT_INT,									// VREF is the source for ref voltage
		adc_mux_1.adc_muxcal = MUX1_MUXCAL_NORM										// Normal operation
	};

	adc_sys0_register adc_sys_0 =
	{
		adc_sys_0.adc_pga_gain = ADC_GAIN_1,
		adc_sys_0.adc_data_rate = data_rate
	};

	adc_mux0_ptr = &adc_mux_0;
	adc_mux1_ptr = &adc_mux_1;
	adc_sys0_ptr = &adc_sys_0;

	adcWorkInProgress = TRUE;														// Lock device for ADC operation

	P9OUT |= ADC_START;																// START pin must be taken high before communicating with registers

	do
	{
		status = adc_spi_wreg_first4(adc_mux0_ptr, adc_mux1_ptr, adc_sys0_ptr);		// Write the first 4 registers simultaneously to avoid 64 clocks requirement between individual register writes
		num_write++;
	}
	while ((status == WRITE_FAIL) && (num_write <= NUM_WRITE));						// If retrying, Tcspw is ok

	P9OUT &= ~ADC_START;															// START pin low to put ADC into sleep mode after the conversion is finished

	LPM3;																			// Wake upon DRDY* interrupt, the conversion result is discarded

	adcWorkInProgress = FALSE;														// Release device for other operations

	if(!USBconnected)
	{
		if (status == WRITE_SUCCESS)
			return adc_measuring;
		else
			return dev_malfunction;
	}
	else
	{
		usb_exit=exit3;
		return usb_connected;
	}
}


void adc_spi_rdata_once (void)
{
	uint8_t i;

	P5OUT &= ~ADC_CS_N;																// Set ADC_CS* LOW to enable SPI communication with ADC

	for (i=0; i < CONV_LENGTH; i++)
	{
		while (!(UCA2IFG & UCTXIFG));												// Wait until TX is done
		UCA2TXBUF = ADC_NOP;														// Send NOP
		while (!(UCA2IFG & UCRXIFG));												// Wait until RX is done
		*(adc_conv_result + i) = UCA2RXBUF;											// Read from RX buffer to clear the RXIFG
	}

	__delay_cycles(30);																// Make sure Tsccs > 1.7 us

	P5OUT |= ADC_CS_N;																// Set ADC_CS* HIGH to disable SPI communication with ADC

}


uint16_t adc_measure (uint32_t num_conv,                             // num_conv depends on RAM storage size.
                      uint8_t *storage,                              // RAM storage for conversion results in bytes.
                      uint16_t index)                                // for the very first conversion index should be 0
{
	uint8_t k;
	uint8_t i;

//	adcWorkInProgress = TRUE; 	// Will be setting this flag externally to account for sleep during settling

	// The conversion results are read in continuous mode (START is high for the duration of measurement)
	P9OUT |= ADC_START;																// START conversion

	for (i=0; i < num_conv; i++)
	{
		if (i == (num_conv - 1))												    // Check if last conversion
		{
			__delay_cycles(30);														// Make sure min Tstart > 3tosc or 0.73 us
			P9OUT &= ~ADC_START;													// START pin should be set low before the last conversion's DRDY* interrupt
		}

		LPM3;																		// Wait until current conversion is over, will return from DRDY* ISR

		adc_spi_rdata_once();														// Retrieve the conversion result

		for (k=0; k < CONV_LENGTH; k++)												// Store result in RAM storage
		{
			*(storage + index) = *(adc_conv_result + k);
			index++;															    // Next time the first conv result byte will be stored in the third element of RAM storage
		}
	}

//	adcWorkInProgress = FALSE;

	return index;                                                                   // If this function is used for one conversion at a time,
	                                                                                // then it will communicate info about the index in ram_stg
	                                                                                // where to store next conversion in the next function call.
}


uint16_t adc_measure_all_channels(uint8_t *storage,
								  uint16_t index,
								  uint16_t *crc)
{
	uint8_t k;

	adcWorkInProgress = TRUE;

	for (k = 0; k < NUM_CHANNELS; k++)                                     	// Repeat measurement for 16 channels
    {
        switch_mux_ch(k+1);

        index = adc_measure(SINGLE_CONV,
                            storage,
                            index);
    }

	adcWorkInProgress = FALSE;

	disable_mux_adg706();													// Disable mux to conserve power

	*crc = crc8_set((storage+index-CONV_LENGTH_16CH),
                         CONV_LENGTH_16CH);                                	// Generate CRC of 16 measurements

    int_divide(*crc,(storage+index));             							// Divide crc_value into two bytes, store in RAM storage after 16 measurements

    index = index + CRC_NUM_BYTES;                                         	// Index points to the element after CRC

    return index;
}


uint8_t adc_sysgcal (uint8_t data_rate)
{
	uint8_t status = 0;
	uint8_t num_write = 0;
	uint8_t adc_reg = 0;

	adc_mux1_register adc_mux_1 =
	{
		adc_mux_1.adc_clkstat = MUX1_CLKSTAT_INT_OSC,								// Internal oscillator
		adc_mux_1.adc_vrefcon = MUX1_VREFCON_VREF_ALT,								// VREF is on/off when necessary
		adc_mux_1.adc_refselt = MUX1_REFSELT_INT_REF0,								// VREF is connected to REF0
		adc_mux_1.adc_muxcal = MUX1_MUXCAL_GAIN_CAL									// Gain calibration
	};

	adc_sys0_register adc_sys_0 =
	{
		adc_sys_0.adc_pga_gain = ADC_GAIN_1,
		adc_sys_0.adc_data_rate = data_rate
	};

	adc_mux1_ptr = &adc_mux_1;
	adc_sys0_ptr = &adc_sys_0;

	adcWorkInProgress = TRUE;

	P9OUT |= ADC_START;																// START pin must be taken high before communicating with registers

	do
	{
		status = adc_spi_wreg_mux1_sys0(adc_mux1_ptr, adc_sys0_ptr);				// Write both mux1 and sys0 simultaneously
		num_write++;
	}
	while ((status == WRITE_FAIL) && (num_write <= NUM_WRITE));						// With retry Tcspw should still be ok

	if (status == WRITE_SUCCESS)
	{
		__delay_cycles(20);															// Make sure Tcspw > 1.22 us

		P5OUT &= ~ADC_CS_N;															// Set ADC_CS* LOW to enable SPI communication with ADC

		while (!(UCA2IFG & UCTXIFG));												// Wait until TX is done
			UCA2TXBUF = adc_wreg_sysgcal;											// Send sysgcal command
		while (!(UCA2IFG & UCRXIFG));												// Wait until RX is done
			adc_reg = UCA2RXBUF;													// Read from RX buffer to clear the RXIFG

		__delay_cycles(30);															// Make sure that Tsccs > 1.7 us
		P5OUT |= ADC_CS_N;															// Set ADC_CS* HIGH to disable SPI communication with ADC
		P9OUT &= ~ADC_START;														// Set START low to place ADS into sleep mode when finish calibration

		LPM3;																		// Wait until calibration is done in LPM3, wake upon DRDY* interrupt

		adcWorkInProgress = FALSE;

		return STATUS_SUCCESS;														// sysgcal is successfully accomplished
	}
	else
	{
		P9OUT &= ~ADC_START;														// Set START low to place ADS into sleep mode when finish calibration

		LPM3;																		// Wake upon DRDY* interrupt

		adcWorkInProgress = FALSE;

		return STATUS_FAIL;
	}
}


void adc_selfocal (void)
{
	uint8_t adc_reg = 0;

	adcWorkInProgress = TRUE;

	P9OUT |= ADC_START;																// START pin must be taken high before communicating with registers, page 33 ADS1247 UGS
	P5OUT &= ~ADC_CS_N;																// Set ADC_CS* LOW to enable SPI communication with ADC

	// send selfocal
	while (!(UCA2IFG & UCTXIFG));													// Wait until TX is done
		UCA2TXBUF = adc_wreg_selfocal;												// Send command bytes
	while (!(UCA2IFG & UCRXIFG));													// Wait until RX is done
		adc_reg = UCA2RXBUF;														// Read from RX buffer to clear the RXIFG

	__delay_cycles(20);																// Make sure that Tsccs > 1.7 us

	P5OUT |= ADC_CS_N;																// Set ADC_CS* HIGH to disable SPI communication with ADC
	P9OUT &= ~ADC_START;															// Set START low to place ADS into sleep mode when finish calibration

	LPM3;																			// Wait until calibration is done in LPM3, wake upon DRDY* goes low in the DRDY* ISR and exit

	adcWorkInProgress = FALSE;
}


void adc_sysocal (void)
{
	uint8_t adc_reg = 0;

	adcWorkInProgress = TRUE;

	P9OUT |= ADC_START;																// START pin must be taken high before communicating with registers, page 33 ADS1247 UGS
	P5OUT &= ~ADC_CS_N;																// Set ADC_CS* LOW to enable SPI communication with ADC

	// send sysocal
	while (!(UCA2IFG & UCTXIFG));													// Wait until TX is done
		UCA2TXBUF = adc_wreg_sysocal;												// Send command bytes
	while (!(UCA2IFG & UCRXIFG));													// Wait until RX is done
		adc_reg = UCA2RXBUF;														// Read from RX buffer to clear the RXIFG

	__delay_cycles(20);																// Make sure that Tsccs > 1.7 us

	P5OUT |= ADC_CS_N;																// Set ADC_CS* HIGH to disable SPI communication with ADC
	P9OUT &= ~ADC_START;															// Set START low to place ADS into sleep mode when finish calibration

	LPM3;																			// Wait until calibration is done in LPM3, wake upon DRDY* goes low in the DRDY* ISR and exit

	adcWorkInProgress = FALSE;
}

device_state adc_calibration (uint8_t data_rate)
{
	uint8_t status = 0;

	status = adc_sysgcal(data_rate);												// System gain calibration

	if (!USBconnected)
	{
		if (status == STATUS_SUCCESS)												// If successful, proceed to self offset calibration
		{
			if (sysocal_enable == TRUE)
				adc_sysocal();														// System offset calibration
			else
				adc_selfocal();														// Self offset calibration

			if (!USBconnected)
				return adc_set;
			else
			{
				usb_exit=exit2;
				return usb_connected;
			}
		}
		else
			return dev_malfunction;
	}
	else
	{
		usb_exit=exit1;
		return usb_connected;
	}
}

