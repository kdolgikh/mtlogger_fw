/*
 * adc_driver.h
 *
 *  Created on: Dec 2, 2016
 *      Author: kdolgikh
 */

/**
 * 	\file adc_driver.h
 * 	\brief TI ADS1247 driver
 */

#ifndef ADC_DRIVER_H_
#define ADC_DRIVER_H_

#include "msp430.h"
#include "stdint.h"
#include "states.h"

/// \name Number of ADC conversions
//@{

#define SETTL_TIME              4           // Number of Sleep_Timer_Fast cycles to make 1 ms of settling time
#define	ADC_NUM_REGISTERS		15			// Number of ADC registers
#define NUM_CHANNELS			16			// Number of ADC channels
#define NUM_WRITE				2			// Number of ADC write register retries
//@}


/// \name Pins for SPI communication with ADC
//@{
#define ADC_CS_N BIT2
#define ADC_RST_N BIT3
#define ADC_START BIT0
//@}


/// \name ADS1247 single byte commands
//@{
#define ADC_WAKEUP			0x01
#define	ADC_SLEEP			0x02
#define	ADC_RESET			0x06
#define ADC_NOP				0xFF
#define ADC_RDATA			0x12
#define ADC_RDATAC			0x14
#define ADC_SDATAC			0x16
#define ADC_SYSOCAL			0x60
#define ADC_SYSGCAL			0x61
#define ADC_SELFOCAL		0x62
//@}


/// \name The first byte of the RREG commands:
//@{
#define ADC_RREG_MUX0_BYTE1			0x20 // start reading from the first register
#define ADC_RREG_VBIAS_BYTE1		0x21
#define ADC_RREG_MUX1_BYTE1			0x22
#define ADC_RREG_SYS0_BYTE1			0x23
#define ADC_RREG_OFC0_BYTE1			0x24
#define ADC_RREG_OFC1_BYTE1			0x25
#define ADC_RREG_OFC2_BYTE1			0x26
#define ADC_RREG_FSC0_BYTE1			0x27
#define ADC_RREG_FSC1_BYTE1			0x28
#define ADC_RREG_FSC2_BYTE1			0x29
#define ADC_RREG_IDAC0_BYTE1		0x2A
#define ADC_RREG_IDAC1_BYTE1		0x2B
#define ADC_RREG_GPIOCFG_BYTE1		0x2C
#define ADC_RREG_GPIODIR_BYTE1		0x2D
#define ADC_RREG_GPIODAT_BYTE1		0x2E // start reading from the last register
//@}

/**
 * 	Note. The following registers don't require writing and stay unmodified:
 *	VBIAS, OFC0-2, FSC0-2, IDAC0-1, all GPIO registers.
 */
/// \name The first byte of the WREG commands
//@{
#define ADC_WREG_MUX0_BYTE1			0x40
#define ADC_WREG_MUX1_BYTE1			0x42
#define ADC_WREG_SYS0_BYTE1			0x43
//@}


/// \name The second byte of the RREG/WREG commands
//@{
#define ADC_BYTE2_SINGLE_REG			0x00 // read/write single register
#define ADC_BYTE2_TWO_REG				0x01 // read/write two registers
#define ADC_BYTE2_THREE_REG				0x02 // read/write three registers
#define ADC_BYTE2_FOUR_REG				0x03 // read/write four registers
#define ADC_BYTE2_ALL_REG				0x0D // read/write all registers
//@}


/// \name Lengths of a conversion result, commands and calibration registers
//@{
#define CONV_LENGTH					3		// Length of a single conversion result, byte
#define CAL_REG_LGTH				3		// Length of a calibration register, byte
#define FOUR_REG_LGTH				4		// Length of three registers mux0, vbias, mux1, sys0
#define SINGLE_REG_RW_CMD_LGTH		3		// Length of the command to read/write a single register, byte
#define TWO_REG_RW_CMD_LGTH			4		// Length of the command to read/write two registers, byte
#define THREE_REG_RW_CMD_LGTH		5		// Length of the command to read/write three registers, byte
#define FOUR_REG_RW_CMD_LGTH		6		// Length of the command to read/write four registers, byte
#define ALL_REG_R_CMD_LGTH			17		// Length of the command to read all registers, byte
//@}


extern uint8_t adcWorkInProgress;


/**
 * ADC data rate
 *
 * Not real value but rather the value of the sys0 register that defines the data rate
 */
extern uint8_t adc_data_rate;


/**
 * Value of the ADC vrefcon field in mux1 register
 */
extern uint8_t adc_mux1_vrefcon;

/**
 * ADC commands to read a single register
 */
extern uint8_t adc_rreg_mux0[SINGLE_REG_RW_CMD_LGTH],
				adc_rreg_vbias[SINGLE_REG_RW_CMD_LGTH],
				adc_rreg_mux1[SINGLE_REG_RW_CMD_LGTH],
				adc_rreg_sys0[SINGLE_REG_RW_CMD_LGTH],
				adc_rreg_idac0[SINGLE_REG_RW_CMD_LGTH],
				adc_rreg_idac1[SINGLE_REG_RW_CMD_LGTH],
				adc_rreg_gpiocfg[SINGLE_REG_RW_CMD_LGTH],
				adc_rreg_gpiodir[SINGLE_REG_RW_CMD_LGTH],
				adc_rreg_gpiodat[SINGLE_REG_RW_CMD_LGTH];

/**
 * ADC commands to read calibration registers
 */
extern uint8_t adc_rreg_ofc[THREE_REG_RW_CMD_LGTH],
				adc_rreg_fsc[THREE_REG_RW_CMD_LGTH];

/**
 * Storage for a calibration register
 */
extern uint8_t adc_calibration_register[CAL_REG_LGTH];


/**
 * If set, sysocal calibration is enabled in the calibration function
 */
extern uint8_t sysocal_enable;


/**
 * ADC command to read all registers
 */
extern const uint8_t adc_rreg_all[ALL_REG_R_CMD_LGTH];


/**
 * Storage for all ADC registers
 */
extern uint8_t adc_regs_all[ADC_NUM_REGISTERS];


/**
 * Array for any single write WREG command
 */
extern uint8_t adc_wreg_single[SINGLE_REG_RW_CMD_LGTH];


/**
 * Array for a single conversion
 */
extern uint8_t adc_conv_result [CONV_LENGTH];


/**
 * ADC calibration commands
 */
extern const uint8_t adc_wreg_sysgcal, adc_wreg_selfocal, adc_wreg_sysocal;


/// \name Positions of bytes for a single read/write and multiple read commands
//@{
#define COMMAND_BYTE_2		1
#define COMMAND_BYTE_3		2
#define COMMAND_BYTE_4		3
#define COMMAND_BYTE_5		4
#define COMMAND_BYTE_6		5
#define COMMAND_BYTE_17		16
//@}


/**
 * 	A struct for ADS1247 MUX0 register
 *
 * 	Note. For current VE and CE schematics, only adc_bcs field might be used for a detection
 * 	of a failed sensor. Analog inputs do not change.
 */
typedef struct adc_mux0_reg
{
	//!	The magnitude of the sensor detect current source
	//!	\n Valid values:
	//!	- \b MUX0_BCS_OFF		 			- burnout current source is off
	//!	- \b MUX0_BCS_05uA		 			- current source is on, 0.5 uA
	//!	- \b MUX0_BCS_2uA		 			- current source is on, 2 uA
	//!	- \b MUX0_BCS_10uA		 			- current source is on, 10 uA
	uint8_t adc_bcs;

	//!	Positive input channel
	//!	\n Valid values:
	//!	- \b AIN0							- default
	//!	- \b AIN1
	//!	- \b AIN2
	//!	- \b AIN3
	uint8_t adc_mux_pos_input;

	//!	Negative input channel
	//!	\n Valid values:
	//!	- \b AIN0
	//!	- \b AIN1							- default
	//!	- \b AIN2
	//!	- \b AIN3
	uint8_t adc_mux_neg_input;
} adc_mux0_register;


/// \name Valid values for the ADS1247 MUX0 register
//@{
#define MUX0_BCS_OFF				0x00
#define MUX0_BCS_05uA				0x40
#define MUX0_BCS_2uA				0x80
#define MUX0_BCS_10uA				0xC0
#define SP_AIN0						0x00
#define SP_AIN1						0x08
#define SP_AIN2						0x10
#define SP_AIN3						0x18
#define SN_AIN0						0x00
#define SN_AIN1						0x01
#define SN_AIN2						0x02
#define SN_AIN3						0x03
//@}


/**
 * 	A struct for ADS1247 MUX1 register
 */
typedef struct adc_mux1_reg
{
	//!	Indicates the type of oscillator being used
	//!	\n Valid values:
	//!	- \b MUX1_CLKSTAT_INT_OSC 	- Internal oscillator
	//!	- \b MUX1_CLKSTAT_EXT_OSC 	- External oscillator
	uint8_t adc_clkstat;

	//!	Internal voltage reference control
	//!	\n Valid values:
	//!	- \b MUX1_VREFCON_VREF_OFF	- Internal reference is always off (default)
	//!	- \b MUX1_VREFCON_VREF_ON	- Internal reference is always on
	//!	- \b MUX1_VREFCON_VREF_ALT	- Internal reference is on when a conversion is in progress and shuts down when the
	//!								  device receives a shutdown opcode or the START pin is taken low
	uint8_t adc_vrefcon;

	//!	Select the reference input for ADC
	//!	\n Valid values:
	//!	- \b MUX1_REFSELT_REF0		- REF0 input pair selected (default)
	//!	- \b MUX1_REFSELT_INT		- Internal reference selected
	//!	- \b MUX1_REFSELT_INT_REF0	- Internal reference selected and internally connected to REF0 input pair
	uint8_t adc_refselt;

	//!	ADC calibration control
	//!	\n Valid values:
	//!	- \b MUX1_MUXCAL_NORM				- Normal operation (default)
	//!	- \b MUX1_MUXCAL_OFFSET_CAL			- Offset measurement
	//!	- \b MUXCAL_GAIN_CAL				- Gain measurement
	//!	- \b MUX1_MUXCAL_TEMP_MEAS			- Temperature diode
	//!	- \b MUX1_MUXCAL_EXT_REF0_MEAS		- External REF0 measurement
	//!	- \b MUX1_MUXCAL_AVDD_MEAS			- AVDD measurement
	//!	- \b MUX1_MUXCAL_DVDD_MEAS			- DVDD measurement
	uint8_t adc_muxcal;
} adc_mux1_register;


/// \name Valid values for the ADS1247 MUX1 register, see page 43 datasheet
//@{
#define MUX1_CLKSTAT_INT_OSC		0x00
#define MUX1_CLKSTAT_EXT_OSC		0x80
#define MUX1_VREFCON_VREF_OFF		0x00
#define MUX1_VREFCON_VREF_ON		0x20
#define MUX1_VREFCON_VREF_ALT		0x40
#define MUX1_REFSELT_REF0			0x00
#define MUX1_REFSELT_INT			0x10
#define MUX1_REFSELT_INT_REF0		0x18
#define MUX1_MUXCAL_NORM			0x00
#define MUX1_MUXCAL_OFFSET_CAL		0x01
#define MUX1_MUXCAL_GAIN_CAL		0x02
#define MUX1_MUXCAL_TEMP_MEAS		0x03
#define MUX1_MUXCAL_EXT_REF0_MEAS	0x05
#define MUX1_MUXCAL_AVDD_MEAS		0x06
#define MUX1_MUXCAL_DVDD_MEAS		0x07
//@}


/**
 * 	A struct for ADS1247 SYS0 register
 */
typedef struct adc_sys0_reg
{
	//!	ADC PGA gain setting
	//!	\n Valid values:
	//!	- \b ADC_GAIN_1			- Gain equals 1
	//!	- \b ADC_GAIN_2			- Gain equals 2
	//!	- \b ADC_GAIN_4			- Gain equals 4
	//!	- \b ADC_GAIN_8			- Gain equals 8
	//!	- \b ADC_GAIN_16		- Gain equals 16
	//!	- \b ADC_GAIN_32		- Gain equals 32
	//!	- \b ADC_GAIN_64		- Gain equals 64
	uint8_t adc_pga_gain;

	//!	ADC output data rate setting
	//!	\n Valid values:
	//!	- \b ADC_DR_5SPS		- 5 SPS
	//!	- \b ADC_DR_10SPS		- 10 SPS
	//!	- \b ADC_DR_20SPS		- 20 SPS
	//!	- \b ADC_DR_40SPS		- 40 SPS
	//!	- \b ADC_DR_80SPS		- 80 SPS
	//!	- \b ADC_DR_160SPS		- 160 SPS
	//!	- \b ADC_DR_320SPS		- 320 SPS
	//!	- \b ADC_DR_640SPS		- 640 SPS
	//!	- \b ADC_DR_1000SPS		- 1000 SPS
	//!	- \b ADC_DR_2000SPS		- 2000 SPS
	uint8_t adc_data_rate;
} adc_sys0_register;


/// \name Valid values for the ADS1247 SYS0 register, gain setting
//@{
#define ADC_GAIN_1 			0x00
#define ADC_GAIN_2 			0x10
#define ADC_GAIN_4			0x20
#define ADC_GAIN_8			0x30
#define ADC_GAIN_16			0x40
#define ADC_GAIN_32			0x50
#define ADC_GAIN_64			0x60
//@}


/// \name Valid values for the ADS1247 SYS0 register, output data rate setting
//@{
#define ADC_DR_5SPS			0x00
#define ADC_DR_10SPS		0x01
#define ADC_DR_20SPS		0x02
#define ADC_DR_40SPS		0x03
#define ADC_DR_80SPS		0x04
#define ADC_DR_160SPS		0x05
#define ADC_DR_320SPS		0x06
#define ADC_DR_640SPS		0x07
#define ADC_DR_1000SPS		0x08
#define ADC_DR_2000SPS		0x09
//@}


/**
* Pointers to structures of type adc_mux0_reg, adc_mux1_reg and adc_sys0_reg
*/
extern adc_mux0_register *adc_mux0_ptr;
extern adc_mux1_register *adc_mux1_ptr;
extern adc_sys0_register *adc_sys0_ptr;


/**
*	\brief Initialize pins for interfacing with ADC
*
*	ADS1247 - MSP430 pin-out:
 * 	MCU:	ADC:
 * 	P1.5	DRDY*
 * 	P5.2	CS*
 * 	P5.3	RST*
 * 	P9.0	START
 * 	P9.1	SCLK // USCI A2
 * 	P9.2	DIN  // USCI A2
 * 	P9.3	DOUT // USCI A2
*/
void adc_spi_pins_init (void);


/**
 *	\brief Initialize SPI communication with ADC
 */
void adc_spi_init (void);


/**
 * 	\brief Reset ADS1247
 *
 * 	@return	Returns adc_set state
 */
device_state adc_reset (void);


/**
 * 	\brief Read single register from ADS1247
 *
 * 	@param 	*adc_rreg_cmd		The first element of the ADC rreg command
 * 	@return	adc_register		Register value returned by the ADC
 */
uint8_t adc_spi_rreg_single (uint8_t *adc_rreg_cmd);


void adc_spi_rreg_first4 (uint8_t *adc_rreg_cmd, uint8_t *four_reg_stg);


/**
 * 	\brief Read 3 bytes of calibration registers OFC or FSC
 *
 * 	@param	*cal_reg_cmd		Pointer to the read calibration register command
 * 	@param	*adc_cal_register	Pointer to the storage for the calibration register
 */
void adc_spi_rreg_cal_register (uint8_t *cal_reg_cmd, uint8_t *adc_cal_register);

/**
 * 	\brief Read all registers from ADS1247
 */
void adc_spi_rreg_all (uint8_t *adc_regs_all);


/**
 * 	\brief Defines the contents of the write command for the ADC MUX0 register
 *
 *	The first byte defines a type of a write command, the second byte defines
 * 	number of register to write, the third byte defines the register value to be
 * 	written.
 *
 * 	@param  *adc_mux0_param		pointer to the struct containing MUX0 parameters
 */
void adc_spi_define_wreg_mux0 (adc_mux0_register *adc_mux0_param);


/**
 * 	\brief Defines the contents of the write command for the ADC MUX1 register
 *
 * 	The first byte defines a type of a write command, the second byte defines
 * 	number of register to write, the third byte defines the register value to be
 * 	written.
 *
 * 	@param  *adc_mux1_param		pointer to the struct containing MUX1 parameters
 */
void adc_spi_define_wreg_mux1 (adc_mux1_register *adc_mux1_param);


/**
 * 	\brief Defines the contents of the write command for the ADC SYS0 register
 *
 * 	The first byte defines a type of a write command, the second byte defines
 * 	number of register to write, the third byte defines the register value to be
 * 	written.
 *
 * 	@param  *adc_mux0_param		pointer to the struct containing SYS0 parameters
 */
void adc_spi_define_wreg_sys0 (adc_sys0_register *adc_sys0_param);


/**
 * 	\brief Write single register (either mux0, mux1 or sys0) to ADS1247
 *
 *	The function accesses the adc_wreg_single array which is prepopulated by adc_spi_define_wreg_xxx command,
 *	that is no parameters are passed to the function.
 *
 * 	@return	status				returned status is success or fail
 */
uint8_t adc_spi_wreg_single (void);


/**
 *	\brief	Block-write registers mux1 and sys0
 *
 *	when performing multiple individual write commands to the first four registers,
 * 	wait at least 64 oscillator clocks before initiating another write command,
 * 	which translates into 32 us or 640 MCLK cycles. To alleviate it, the command to make
 * 	a block write of mux1 and sys0 is used.
 *
 *	@param	*adc_mux1_param		ptr to the struct containing mux1 register parameters
 *	@param	*adc_sys0_param		ptr to the struct containing sys0 register parameters
 * 	@return	status				returned status is success or fail
 */
uint8_t adc_spi_wreg_mux1_sys0 (adc_mux1_register *adc_mux1_param, adc_sys0_register *adc_sys0_param);


uint8_t adc_spi_wreg_mux0 (void);


/**
 * \brief Set up ADC parameters for measurements
 *
 *	This function discards the first conversion after the set up.
 *
 *	In future the function should have ADC parameters as the input.
 *
 *
 * @param	data_rate	 		Value of the sys0 register that corresponds to specific data rate
 * @param	mux1_vrefcon		Value of the vrefcon field in mux1 register; controls VREF module
 * @return	device_state 		Return state adc_measuring
 */
device_state adc_setup (uint8_t data_rate, uint8_t mux1_vrefcon);


/**
 * 	\brief Receive one conversion result over SPI
 *
 */
void adc_spi_rdata_once (void);


/**
 * 	\brief Perform self-offset calibration of ADS1247
 */
void adc_selfocal (void);

/**
 * 	\brief Perform system offset calibration
 */
void adc_sysocal (void);


/**
 * 	\brief Perform system gain calibration of ADS1247
 *
 *	@param	data_rate			Value of the sys0 register that corresponds to specific data rate
 * 	@return	status				Returned status is success or fail
 */
uint8_t adc_sysgcal (uint8_t data_rate);


/**
 * 	\brief Performs the ADC's calibration
 *
 *	@param	data_rate			Value of the sys0 register that corresponds to specific data rate
 * 	@return	device_state		Returned device_state is adc_set if success or adc_broken if fail
 */
device_state adc_calibration (uint8_t data_rate);


/**
 * 	\brief	Function that performs ADC measurements for a single channel
 *
 *  Num_conv depends on RAM storage size. Function stores conv results directly into an array
 *  rather than making this afterwards in another function. This allows populating arrays of, for example,
 *  flash segment size and write data to flash.
 *
 *  Index allows populating RAM storage that has a bigger size than is necessary based on current num_conv.
 *  E.g. array can hold 100 conversions, and num_conv is 10. The function can be called with previously returned
 *  index value to append new data to the existing data in the storage.
 *
 *  This function is also useful for averaging of measurements from single channel.
 *
 * 	@param	num_conv            Number of conversions to make
 * 	@param	*storage			Pointer to storage for the conv results (in RAM). Results are returned in bytes.
 * 	@param	index				Index that indicates the position of the next conversion result in ram storage
 * 	@return index
 *
 */
uint16_t adc_measure (uint32_t num_conv, uint8_t *storage, uint16_t index);


uint16_t adc_measure_all_channels (uint8_t *storage, uint16_t index, uint16_t *crc);

#endif /* ADC_DRIVER_H_ */
