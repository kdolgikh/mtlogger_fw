/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
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

#include "driverlib.h"
#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                 // USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"
#include "mux_adg706_driver.h"
#include "adc_driver.h"
#include "states.h"
#include "init.h"
#include "send_over_usb.h"
#include "sleep_timer.h"
#include "sdcard.h"
#include "measure.h"
#include "real_time_clock.h"

device_state dev_state = adc_measuring;					// Device state, start with calibration

//volatile uint16_t num_drdy = 0;
//volatile uint8_t ButtonPressed_event = FALSE;		// Flag that prevents ADC operation until cleared in the user button's ISR
//volatile uint32_t ta0_overflow = 0;                 // Overflows of TA0 used to measure duration of certain operations

uint32_t max_segments_sd = 0;
//volatile uint8_t flag_overflow = 0;
//volatile uint8_t flag_ccr = 0;


/*  
 * ======== main ========
 */
void main (void)
{
	device_init();				                            // Init device modules
    max_segments_sd = num_segments_sdc(sdcSize);            // Determine max num of segments on SD-card

    __enable_interrupt();  		                            // Enable interrupts globally

    // These two functions need Sleep_Timer_Fast which relies on interrupts
    Radio_Init();

    USB_setup(FALSE, TRUE); 		                        // Init USB & events

    while (1)
    {
        switch (USB_getConnectionState())
        {
            case ST_ENUM_ACTIVE: //4
                if (USB_DataReceived_event == TRUE)
            	{
                	USB_DataReceived_event = FALSE;
                	usb_command_line();
            	}
                if(USBconnected)
                	LPM0;	// wait until data is received
            	break;

            // USB disconnected
            case ST_PHYS_DISCONNECTED: //1
//            	num_drdy=0;
            	rt1ps_count=0;

            	if(start_immediately)		// Start only when user commands it
            	{
            		RTCPS1CTL &= ~RT1PSIFG;	// clear interrupt flag
            		RTCPS1CTL |= RT1PSIE;	// Enable RT1PS interrupt here to sync the measurement
            	}

            	LPM3;						// Wait until RT1PSIFG is hit or sleep here until USB wakes the device

				if (dev_state == adc_measuring)
					if (!CalEn)
					{
						adc_calibration(adc_data_rate);		// Calibration is performed with current data rate
						adc_setup(adc_data_rate, adc_mux1_vrefcon);
					}
					dev_state = measure_all_channels_store_flash_sdc();
				if (dev_state == adc_rst)
					dev_state = adc_reset();
				if (dev_state == usb_connected)
					usb_connect2host();
				if (dev_state == parameter_error)
					_NOP();
				if (dev_state == dev_mem_full)
					_NOP();
				if (dev_state == dev_malfunction)
					_NOP();
				break;

			// These cases are executed while the device is enumerated but suspended
			// by the host, or connected to a powered hub without a USB host
			// present.
            case ST_PHYS_CONNECTED_NOENUM: //2
            case ST_ENUM_SUSPENDED: //5
            case ST_PHYS_CONNECTED_NOENUM_SUSP: //6
            	LPM3;
                break;

            // The default is executed for the momentary state
            // ST_ENUM_IN_PROGRESS.  Usually, this state only last a few
            // seconds.  Be sure not to enter LPM3 in this state; USB
            // communication is taking place here, and therefore the mode must
            // be LPM0 or active-CPU.
            case ST_ENUM_IN_PROGRESS: //3
            default:;
        }

    }  //while(1)
} //main()


#pragma vector=RTC_VECTOR
__interrupt void RTC_B_ISR (void) {
    switch (RTCIV) {
        case (RTCIV_RTCRDYIFG):
			RTCCTL0 &= ~RTCRDYIE;		// Disable RTC Ready interrupt
            LPM0_EXIT;
            break;
        case(RTCIV_RTCAIFG):
        	break;
        case(RTCIV_RT0PSIFG):
        	break;
        case(RTCIV_RT1PSIFG):
        	if (start_immediately)		// Sync measurement with 2 sec boundary
        	{
        		start_immediately = FALSE;
        		LPM3_EXIT;				// Used only once
        	}
        	else
        	{   // Count interrupts to calculate meas rate
				rt1ps_count++;	// volatile
				if (rt1ps_count == (meas_rate >> 1)) // Divide by 2
				{
					rt1ps_count = 0;
					LPM3_EXIT;
				}
        	}
        	break;
        case(RTCIV_RTCOFIFG):
        	// Add procedure to recover from oscillator fault
        	break;
        default:
            break;
    }
}


#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR (void) {
	switch (P1IV) {
			case (P1IV_P1IFG3):	// user button
				break;
			case (P1IV_P1IFG5):
//				num_drdy++;
				switch(dev_state)
				{
					case(adc_set):
					case(adc_cal):
					case(adc_measuring):
						LPM3_EXIT;
						break;
					case(usb_enumerated):
						LPM0_EXIT;
					default:
						break;
				}
				break;
			default:
				break;
			}
}


/*#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void) {
	switch (TA1IV) {
		case (TA1IV_TAIFG):
			timer_event = TRUE; // volatile
			LPM3_EXIT;
			break;
		default:
			break;
	}
}*/


#pragma vector=TIMER2_A1_VECTOR
__interrupt void Timer2_A1_ISR (void) {
	switch (TA2IV) {
		case (TA2IV_TACCR1):
//			flag_ccr++; // volatile
			timer_event = TRUE; // volatile
			LPM3_EXIT;
			break;
		case (TA2IV_TAIFG):
//			flag_overflow++; // volatile
//			Fast_Timer_Rollover_Count++; // volatile
			break;
		default:
			break;
		}
}


/*#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1_ISR (void) {
    switch (TA0IV) {
        case (TA0IV_TAIFG):
            ta0_overflow++; // volatile
            break;
        default:
            break;
    }
}*/



/*  
 * ======== UNMI_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(UNMI_VECTOR))) UNMI_ISR (void)
#else
#error Compiler not found!
#endif
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG ))
    {
        case SYSUNIV_NONE:
            __no_operation();
            break;
        case SYSUNIV_NMIIFG:
            __no_operation();
            break;
        case SYSUNIV_OFIFG:
            UCS_clearFaultFlag(UCS_XT2OFFG);
            UCS_clearFaultFlag(UCS_DCOFFG);
            SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
            break;
        case SYSUNIV_ACCVIFG:
            __no_operation();
            break;
        case SYSUNIV_BUSIFG:
            // If the CPU accesses USB memory while the USB module is
            // suspended, a "bus error" can occur.  This generates an NMI.  If
            // USB is automatically disconnecting in your software, set a
            // breakpoint here and see if execution hits it.  See the
            // Programmer's Guide for more information.
            SYSBERRIV = 0; //clear bus error flag
            USB_disable(); //Disable
    }
}

//Released_Version_5_20_06_02
