/*
 * send_over_usb.h
 *
 *  Created on: Mar 31, 2017
 *      Author: kdolgikh
 */

#ifndef SEND_OVER_USB_H_
#define SEND_OVER_USB_H_

// the first byte of all messages is always an ACK,
// hence all buffer lengths have an increase by 1 from what the data inside needs

#define USB_TX_RETRY_NUM	100					// Number of USB TX retries
#define USB_MSG_TRX_LGTH	USB_RTC_BUFF_LGTH	// Length of the USB control message to/from PC, byte. Max num of bytes that can be sent/received.
#define PARAM_POS			1					// Position of the parameter in the received message. Position 0 is reserved for ACK
#define ACK_BUFF_LGTH		1					// Length of the buffer for ACK
#define PARAM_BUFF_LGTH		2					// Length of the buffer for parameters
#define CAL_REG_BUFF_LGTH	4					// Length of the buffer for calibration register
#define NUM_BYTE_BUFF_LGTH	5					// Length of the buffer for num_byte value
#define MEAS_RATE_BUFF_LGTH	5					// Length of the buffer for meas rate
#define	USB_RTC_BUFF_LGTH	7					// Length of the USB message to/from PC containing RTC data, byte
#define RTC_YEAR_POS		1					// Position of Year in the received message
#define RTC_MON_POS			2					// Position of Month
#define RTC_DAY_POS			3					// Position of Day of Month
#define RTC_HOUR_POS		4					// Position of Hour
#define	RTC_MIN_POS			5					// Position of Minute
#define RTC_SEC_POS			6					// Position of Seconds

/**
 * USB communication commands
 */
#define GET_CONV_RESULTS		0x7F			// Transfer conversion results over USB
#define USB_GET_ADC_DATA_RATE	0x03			// Get ADC data rate
#define	USB_SET_ADC_DATA_RATE	0x05			// Set ADC data rate
#define	USB_GET_NUM_CONV		0x07			// Get number of 170 conversion blocks
#define USB_SET_NUM_CONV		0x09			// Set number of 170 conversion blocks
#define USB_GET_ADC_VREFCON		0x0A			// Get ADC VREFCON value
#define USB_SET_ADC_VREFCON		0x0B			// Set ADC VREFCON value
#define USB_GET_ADC_OFC			0x0C			// Get ADC offset calibration register OFC
#define USB_GET_ADC_FSC			0x0D			// Get ADC gain calibration register FSC
#define USB_GET_DEV_STATE		0x0E			// Get the state of the device
#define USB_SET_DEV_STATE		0x0F			// Set the state of the device
#define USB_GET_RTC				0x11			// Get time over USB
#define USB_SET_RTC				0x12			// Get time over USB
#define USB_GET_SYSOCAL_EN		0x13			// Enable/disable sysocal
#define USB_SET_SYSOCAL_EN		0x14			// Check if sysocal enabled or not
#define USB_GET_DELAY_EN		0x15			// Enable delay before measuring
#define USB_SET_DELAY_EN		0x16			// Check if delay is enabled
#define USB_SET_MEAS_RATE		0x17			// Set measurement rate
#define USB_SET_MEAS_START		0x18			// Set measurement start options
#define USB_GET_MEAS_RATE		0x19			// Get meas rate
#define USB_SET_KEEP_DATA		0x1A			// Set KeepData flag
#define USB_GET_KEEP_DATA		0x1B			// Get KeepData flag
#define USB_SET_CAL_EN			0x1C			// Get Calibration Enabled flag
#define USB_GET_CAL_EN			0x1D			// Set Calibration Enabled flag


#include "states.h"


/**
 * \brief Flag showing the reception of the message from PC
 */
extern volatile uint8_t USB_DataReceived_event;

extern uint8_t USBconnected;


void usb_connect2host (void);


/**
 * \brief Command line interface for the device through USB
 */
void usb_command_line (void);


void usb_ack (void);


/**
 * \brief Send the conversion result that is stored in Flash over USB
 *
 */
void usb_send_conv_result (void);


void usb_get_dev_param (void);


void usb_get_meas_rate(void);


void usb_get_adc_cal_register(void);


void usb_get_rtc(void);


void usb_set_dev_param(void);


void usb_set_adc_vrefcon(void);


void usb_set_rtc(void);


#endif /* SEND_OVER_USB_H_ */
