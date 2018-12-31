/* --COPYRIGHT--,BSD
 * Copyright (c) 2011, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//----------------------------------------------------------------------------
//  Description:  This file contains definitions specific to the CC1100/2500.
//  The configuration registers, strobe commands, and status registers are 
//  defined, as well as some common masks for these registers.
//
//  MSP430/CC1100-2500 Interface Code Library v1.0
//
//  K. Quiring
//  Texas Instruments, Inc.
//  July 2006
//  IAR Embedded Workbench v3.41
//----------------------------------------------------------------------------

/** \file CC110l.h
 * 	\brief Contains definitions for the registers, strobes, and bit masks for the CC110l
 */


/// \name Configuration Registers

//@{
#define IOCFG2       0x00        ///< GDO2 output pin configuration
#define IOCFG1       0x01        ///< GDO1 output pin configuration
#define IOCFG0       0x02        ///< GDO0 output pin configuration
#define FIFOTHR      0x03        ///< RX FIFO and TX FIFO thresholds
#define SYNC1        0x04        ///< Sync word, high byte
#define SYNC0        0x05        ///< Sync word, low byte
#define PKTLEN       0x06        ///< Packet length
#define PKTCTRL1     0x07        ///< Packet automation control
#define PKTCTRL0     0x08        ///< Packet automation control
#define ADDR         0x09        ///< Device address
#define CHANNR       0x0A        ///< Channel number
#define FSCTRL1      0x0B        ///< Frequency synthesizer control
#define FSCTRL0      0x0C        ///< Frequency synthesizer control
#define FREQ2        0x0D        ///< Frequency control word, high byte
#define FREQ1        0x0E        ///< Frequency control word, middle byte
#define FREQ0        0x0F        ///< Frequency control word, low byte
#define MDMCFG4      0x10        ///< Modem configuration
#define MDMCFG3      0x11        ///< Modem configuration
#define MDMCFG2      0x12        ///< Modem configuration
#define MDMCFG1      0x13        ///< Modem configuration
#define MDMCFG0      0x14        ///< Modem configuration
#define DEVIATN      0x15        ///< Modem deviation setting
#define MCSM2        0x16        ///< Main Radio Cntrl State Machine config
#define MCSM1        0x17        ///< Main Radio Cntrl State Machine config
#define MCSM0        0x18        ///< Main Radio Cntrl State Machine config
#define FOCCFG       0x19        ///< Frequency Offset Compensation config
#define BSCFG        0x1A        ///< Bit Synchronization configuration
#define AGCCTRL2     0x1B        ///< AGC control
#define AGCCTRL1     0x1C        ///< AGC control
#define AGCCTRL0     0x1D        ///< AGC control
#define WOREVT1      0x1E        ///< High byte Event 0 timeout
#define WOREVT0      0x1F        ///< Low byte Event 0 timeout
#define WORCTRL      0x20        ///< Wake On Radio control
#define FREND1       0x21        ///< Front end RX configuration
#define FREND0       0x22        ///< Front end TX configuration
#define FSCAL3       0x23        ///< Frequency synthesizer calibration
#define FSCAL2       0x24        ///< Frequency synthesizer calibration
#define FSCAL1       0x25        ///< Frequency synthesizer calibration
#define FSCAL0       0x26        ///< Frequency synthesizer calibration
#define RCCTRL1      0x27        ///< RC oscillator configuration
#define RCCTRL0      0x28        ///< RC oscillator configuration
#define FSTEST       0x29        ///< Frequency synthesizer cal control
#define PTEST        0x2A        ///< Production test
#define AGCTEST      0x2B        ///< AGC test
#define TEST2        0x2C        ///< Various test settings
#define TEST1        0x2D        ///< Various test settings
#define TEST0        0x2E        ///< Various test settings
//@}



/// \name Strobe commands

//@{
#define SRES         0x30        ///< Reset chip.
#define SFSTXON      0x31        ///< Enable/calibrate freq synthesizer
#define SXOFF        0x32        ///< Turn off crystal oscillator.
#define SCAL         0x33        ///< Calibrate freq synthesizer & disable
#define SRX          0x34        ///< Enable RX.
#define STX          0x35        ///< Enable TX.
#define SIDLE        0x36        ///< Exit RX / TX
#define SAFC         0x37        ///< AFC adjustment of freq synthesizer
#define SWOR         0x38        ///< Start automatic RX polling sequence
#define SPWD         0x39        ///< Enter pwr down mode when CSn goes hi
#define SFRX         0x3A        ///< Flush the RX FIFO buffer.
#define SFTX         0x3B        ///< Flush the TX FIFO buffer.
#define SWORRST      0x3C        ///< Reset real time clock.
#define SNOP         0x3D        ///< No operation.
//@}

/// \name Status registers
//@{
#define PARTNUM      0x30        ///< Part number
#define VERSION      0x31        ///< Current version number
#define FREQEST      0x32        ///< Frequency offset estimate
#define LQI          0x33        ///< Demodulator estimate for link quality
#define RSSI         0x34        ///< Received signal strength indication
#define MARCSTATE    0x35        ///< Control state machine state
#define WORTIME1     0x36        ///< High byte of WOR timer
#define WORTIME0     0x37        ///< Low byte of WOR timer
#define PKTSTATUS    0x38        ///< Current GDOx status and packet status
#define VCO_VC_DAC   0x39        ///< Current setting from PLL cal module
#define TXBYTES      0x3A        ///< Underflow and # of bytes in TXFIFO
#define RXBYTES      0x3B        ///< Overflow and # of bytes in RXFIFO

/// \name Masks for packet status register
//@{
#define OverFlow 	0b10000000	 ///< Masks off the overflow bit
#define FIFO_Level	0b01111111	 ///< Masks off the FIFO level

#define NUM_RXBYTES  0x7F        ///< Mask "# of bytes" field in _RXBYTES
//@}

/// \name Other memory locations
//@{
#define PATABLE      0x3E		///< Address for PATABLE
#define TXFIFO       0x3F		///< Address for TX FIFO
#define RXFIFO       0x3F		///< Address for RX FIFO
//@}

/// \name Masks for status bytes
//@{
#define LQI_RX       0x01        ///< Position of LQI byte
#define CRC_OK       0x80        ///< Mask "CRC_OK" bit within LQI byte
#define FIFO_Bytes	 0b00001111	 ///< Mask the number of FIFO bytes in the status byte
#define State_Bits	 0b01110000  ///< Masks off the state bits from the status byte
//@}

/// \name State bit values
//@{
#define SB_Idle		 0b00000000		///< Radio is in idle state
#define SB_RX		 0b00010000		///< Radio is currently receiving
#define SB_TX		 0b00100000		///< Radio is currently transmitting
#define SB_FSTXON	 0b01000000		///< Radio is ready for Fast TX
#define SB_Settle	 0b01010000		///< Radio is waiting for PLL to settle
#define SB_RX_Over	 0b01100000		///< Radio RX Fifo has overflowed
#define SB_TX_Over	 0b01110000		///< Radio TX Fifo has underflowed
//@}
/// \name Definitions to support burst/single access:
//@{
#define WRITE_BURST  0x40 ///< Burst write bits
#define READ_SINGLE  0x80 ///< Read bits
#define READ_BURST   0xC0 ///< Burst read bits
//@}

/// \name Definitions for strobes
//@{
#define Get_TX_FIFO 0x00 ///< Makes strobe return bytes in TX FIFO
#define Get_RX_FIFO 0x80 ///< Makes strobe return bytes in RX FIFO
//@}


