/*
 * states.h
 *
 *  Created on: Dec 5, 2016
 *      Author: kdolgikh
 */

/**
 * \file states.h
 * \brief Contains information on different states of the device and its modules
 */


#ifndef STATES_H_
#define STATES_H_

#define STATUS_INTERRUPTED 	2

#define WRITE_SUCCESS	0x10
#define WRITE_FAIL		0x11

#define CAL_REQD		1
#define CAL_NOT_REQD	0

typedef enum {
	adc_read_single_reg,	///< Read single register
	adc_write_single_reg,	///< Write single register
	adc_read_all_reg,		///< Read all registers
	adc_rx_conv_data		///< Receive conversion data
} adc_comm_state;


typedef enum {
	usb_enumerated,			///< The device is connected to PC and ready to send/receive data over USB
	usb_not_enum,			///< For USB states other than enumerated and suspended
	usb_connected,			///< USB cable has been connected to the device
	adc_set,				///< Set parameters of the ADC
	adc_cal,				///< Calibration is in progress
	adc_measuring,			///< Measure is in progress
	adc_rst,				///< Reset ADC
	dev_mem_full,           ///< Device memory is full
	dev_malfunction,		///< Device broken
	parameter_error,        ///< Error in operational parameter
	usb_reset,				///< Add code to handle USB reset
	usb_suspended			///< The device has been suspended by the host
} device_state;

typedef enum {
	not_exit,
	exit1,
	exit2,
	exit3,
	exit4,
	exit5,
	exit6,
	exit7,
	exit8
} exit_point;


#endif /* STATES_H_ */
