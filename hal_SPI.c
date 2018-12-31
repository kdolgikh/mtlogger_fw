/* ***********************************************************
* THIS PROGRAM IS PROVIDED "AS IS". TI MAKES NO WARRANTIES OR
* REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY, 
* INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS 
* FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR 
* COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE. 
* TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET 
* POSSESSION, AND NON-INFRINGEMENT OF ANY THIRD PARTY 
* INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR 
* YOUR USE OF THE PROGRAM.
*
* IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL, 
* CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY 
* THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED 
* OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT 
* OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM. 
* EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF 
* REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS 
* OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF 
* USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI'S 
* AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF 
* YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS 
* (U.S.$500).
*
* Unless otherwise stated, the Program written and copyrighted 
* by Texas Instruments is distributed as "freeware".  You may, 
* only under TI's copyright in the Program, use and modify the 
* Program without any charge or restriction.  You may 
* distribute to third parties, provided that you transfer a 
* copy of this license to the third party and the third party 
* agrees to these terms by its first use of the Program. You 
* must reproduce the copyright notice and any other legend of 
* ownership on each copy or partial copy, of the Program.
*
* You acknowledge and agree that the Program contains 
* copyrighted material, trade secrets and other TI proprietary 
* information and is protected by copyright laws, 
* international copyright treaties, and trade secret laws, as 
* well as other intellectual property laws.  To protect TI's 
* rights in the Program, you agree not to decompile, reverse 
* engineer, disassemble or otherwise translate any object code 
* versions of the Program to a human-readable form.  You agree 
* that in no event will you alter, remove or destroy any 
* copyright notice included in the Program.  TI reserves all 
* rights not specifically granted under this license. Except 
* as specifically provided herein, nothing in this agreement 
* shall be construed as conferring by implication, estoppel, 
* or otherwise, upon you, any license or other right under any 
* TI patents, copyrights or trade secrets.
* ********************************************************* */


#ifndef _SPILIB_C
#define _SPILIB_C
//
//---------------------------------------------------------------
#include "hal_SPI.h"
#include "driverlib.h"

//Send one byte via SPI
uint8_t spiSendByte(uint8_t data)
{
  while (!(UCB1IFG & UCTXIFG));    // wait while not ready for TX
  	  UCB1TXBUF = data;            // write
  while (!(UCB1IFG & UCRXIFG));    // wait for RX buffer (full)
  	  return (UCB1RXBUF);
}


//Read a frame of bytes via SPI
uint8_t spiReadFrame(uint8_t *pBuffer, uint16_t size)
{
	uint16_t i = 0;

	// Receive data
	for (i = 0; i < size; i++)
	{
		while (!(UCB1IFG & UCTXIFG));   // wait while not ready for TX
			UCB1TXBUF = 0xFF;	     	// dummy write
		while (!(UCB1IFG & UCRXIFG));   // wait for RX buffer (full)
			pBuffer[i] = UCB1RXBUF;
	}
	return(0);
}


//Send a frame of bytes via SPI
uint8_t spiSendFrame(uint8_t *pBuffer, uint16_t size)
{
	uint8_t dummy_byte = 0;

	uint16_t i = 0;
	// clock the actual data transfer and receive the bytes; spi_read automatically finds the Data Block
	for (i = 0; i < size; i++)
	{
		while (!(UCB1IFG & UCTXIFG));   // wait while not ready for TX
			UCB1TXBUF = *(pBuffer+i);   // write
		while (!(UCB1IFG & UCRXIFG));   // wait for RX buffer (full)
			dummy_byte = UCB1RXBUF;
	}

	return dummy_byte;
}

//---------------------------------------------------------------------
#endif /* _SPILIB_C */
