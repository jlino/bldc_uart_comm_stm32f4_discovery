/*
	Copyright 2016 João Lino	jl@joaolino.com

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * comm_uart_arduino.c
 *
 *  Created on: 19 dez 2016
 *      Author: joão
 */

#include <comm_uart_arduino.h>
#include <bldc_interface_uart.h>

// Settings
#define __CUA_UART_BAUDRATE			115200
//#define SERIAL_RX_BUFFER_SIZE	1024/4

// Private functions
static void send_packet(unsigned char *data, unsigned int len);

// Interrupts (Makeshift "Threads")
static bool is_data_ready = false;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
//static bool is_data_ready = false;

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz 
	// Wait for data to become available and process it as long as there is data.
	if(is_data_ready) {
		while (serial_rx_read_pos != serial_rx_write_pos) {
			bldc_interface_uart_process_byte(serial_rx_buffer[serial_rx_read_pos++]);

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}

/**
 * This thread is only for calling the timer function once
 * per millisecond. Can also be implementer using interrupts
 * if no RTOS is available.
 */
 ISR(TIMER2_COMPA_vect){//timer1 interrupt 1kHz
	bldc_interface_uart_run_timer();
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
	while (Serial.available()) {

		serial_rx_buffer[serial_rx_write_pos++] = Serial.read();
		
		if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
			serial_rx_write_pos = 0;
		}
	}

	is_data_ready = true;
}

/**
 * Callback that the packet handler uses to send an assembled packet.
 *
 * @param data
 * Data array pointer
 * @param len
 * Data array length
 */
static void send_packet(unsigned char *data, unsigned int len) {
	if (len > (PACKET_MAX_PL_LEN + 5)) {
		return;
	}

	/* Wait for the previous transmission to finish.
	while (UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}*/

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);

	// Send the data over UART
	Serial.write(buffer, len);
	//uartStartSend(&UART_DEV, len, buffer);
}

 void comm_uart_arduino_init() {
	// Initialize UART
	Serial.begin(__CUA_UART_BAUDRATE);
	/*while (!Serial) {
	    ; // wait for serial port to connect. Needed for native USB port only
	}*/

	// Initialize the bldc interface and provide a send function
	bldc_interface_uart_init(&send_packet);

	//stop interrupts
	noInterrupts();

	/*set timer0 interrupt at 61.1Hz
	TCCR0A = 0;// set entire TCCR0A register to 0
	TCCR0B = 0;// same for TCCR0B
	TCNT0  = 0;//initialize counter value to 0
	// set compare match register for 61.1hz increments
	OCR0A = 255;// = (16*10^6) / (1*1024) - 1 (must be <256)
	// turn on CTC mode
	TCCR0A |= (1 << WGM01);
	// Set CS02 and CS00 bits for 1024 prescaler
	TCCR0B |= (1 << CS02) | (1 << CS00);   
	// enable timer compare interrupt
	TIMSK0 |= (1 << OCIE0A);*/

	//set timer1 interrupt at 1Hz
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
	// set compare match register for 1hz increments
	OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS101 and CS12 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10);  
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	//set timer2 interrupt at 1kHz
	TCCR2A = 0;// set entire TCCR2A register to 0
	TCCR2B = 0;// same for TCCR2B
	TCNT2  = 0;//initialize counter value to 0
	// set compare match register for 1khz increments
	OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
	// turn on CTC mode
	TCCR2A |= (1 << WGM21);
	// Set CS21 bit for 64 prescaler
	TCCR2B |= (1 << CS22);   
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);

	// Inotialize the processing interrupt
	is_data_ready = false;

	// Allow interrupts so that the Processing and Timer interrupts ("Threads") start
	interrupts();
}

/*
ISR(TIMER0_COMPA_vect){//timer0 interrupt 61.1Hz 

}
*/