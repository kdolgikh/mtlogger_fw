/*
 * Spi_Library.h
 *
 *  Created on: Oct 16, 2015
 *      Author: cgoss
 */

/**
 * \file SPI_Library.h
 * \brief Library for communicating with CC110l over SPI
 *
 */

#include <stdint.h>

#ifndef SPI_LIBRARY_H_
#define SPI_LIBRARY_H_

// Outward facing pins/registers for the radio
/// \name GDO Pins on MSP430
//@{
#define MSP_RX_Pin BIT6					///> Pin that the GDO flagging for RX is attached to
#define GDO_RX	IOCFG0					///> Register controlling the GDO pin used to signal RX receive.
#define MSP_RX_Port_OUT P1OUT
#define MSP_RX_Port_IFG	P1IFG			///> Port interrupt flag
#define MSP_RX_Port_IE	P1IE			///> Interrupt enable register
#define MSP_RX_Port_IES P1IES			///> Interrupt edge select
//@}


/**
 * \brief Initialization function for the SPI link
 *
 *	Uses the Port/Pin mapping values in SPI_Pins.h to configure the necessary registers for
 *	SPI communication with the CC110l. These mappings are device specific and would need to be
 *	changed depending on which MSP430 is being targetted, and which USCI module is being used.
 *	The preferred method of handling multiple boards is to use #ifdef blocks targetted at the
 *	define value used with the MSP430.h file.
 *
 */

void SPI_Init(void);

/**
 * \brief A function for sending a single byte to the radio
 *
 *	Writes a byte to the specified register.
 *
 * @param address The register in the CC110l that the byte is to be written to.
 * @param value The value to write to the target register.
 * @return The status byte from the CC110l. For a write operation the 4 LSB of the byte will represent the space left in the TX_FIFO.
 */
uint8_t SPI_Send(uint8_t address, uint8_t value);

/**
 * \brief Reads a single register
 *
 * Fetches the value stored in the specified register. Returns the status byte and copies the register value into the out parameter
 *
 * @param address The register to be read.
 * @param out Pointer to write the stored register value into.
 * @return The status byte from the CC110l, for a read operation the 4 LSB of the byte represent the new data in the RX FIFO.
 */
uint8_t SPI_Read(uint8_t address, uint8_t* out);

/**
 * \brief Makes a burst write into a set of registers, beginning with the register pointed by the address value.
 *
 * The first value in the input array is written into the register at address.
 * The second value is written into the next register, and so on.
 * The status byte is captured from the transmission of the final byte in the burst,
 * and will represent the space left in the TX FIFO just before the final byte is transmitted.
 *
 * @param address The address for the first register to write.
 * @param value An array of values to write into CC110l registers.
 * @param length The number of values that are going to be written to the CC110l.
 * @return The status byte of the CC110l
 */

uint8_t SPI_Send_Burst(uint8_t address, uint8_t* value, uint8_t length);

/**
 * \brief Makes a burst read of the registers in the CC110l.
 *
 * A succession of registers in the CC110l will be copied into the array.
 * The register value at the provided address will be written into the first position
 * of the array, the next register into the second position in the array, and so on.
 *
 * @param address The beginning register address.
 * @param out Pointer to the array the register values are going to be written into.
 * @param length The number of elements to read.
 * @return The CC110l status byte
 */

uint8_t SPI_Read_Burst(uint8_t address, uint8_t* out, uint8_t length);

/**
 * \brief Sends a strobe command to the CC110l
 *
 * The operation of the CC110l is a basic state machine. It will move between
 * states based on either internal events or through strobe commands which are passed to it.
 *
 * @param strobe One of the strobe commands from CC110l.h
 * @param FIFO Get_TX_FIFO or Get_RX_FIFO
 * @return The CC110l status byte
 */
uint8_t SPI_Strobe(uint8_t strobe, uint8_t FIFO);

/**
 * \brief Reads a value from one of the special status registers in the CC110l
 *
 * The CC110l has a set of special read only status registers. They contain
 * information such as the version number of the chip, the current state machine
 * state, and the FIFO buffer levels.
 *
 * @param status_reg The status register to read the value from.
 * @param out A pointer to the value that the status register should be written into.
 * @return The CC110l status byte.
 */
uint8_t SPI_Read_Status(uint8_t status_reg, uint8_t* out);


#endif /* SPI_LIBRARY_H_ */
