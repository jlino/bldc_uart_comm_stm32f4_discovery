/*
 * comm_uart.c
 *
 *  Created on: 19 dez 2016
 *      Author: joão
 */

#ifndef COMM_UART_ARDUINO_H_
#define COMM_UART_ARDUINO_H_

#define __CUA_USE_SECOND_SERIAL

#include "Arduino.h"
#ifdef __CUA_USE_SECOND_SERIAL
	#include <SoftwareSerial.h>
#else
	#define mySerial Serial
#endif

//#define DEBUG_BLDC

// Functions
void comm_uart_arduino_init();

void softwareSerialEvent();

#endif /* COMM_UART_ARDUINO_H_ */