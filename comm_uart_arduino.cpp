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
#define __CUA_UART_BAUDRATE			115200 // 9600

// Private functions
static void send_packet(unsigned char *data, int len);

// Interrupts (Makeshift "Threads")
static bool is_data_ready = false;

#ifdef __CUA_USE_SECOND_SERIAL
	SoftwareSerial mySerial(8, 7); // RX, TX
#endif

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
void softwareSerialEvent() {
	bool isWork = false;
	#ifdef DEBUG_BLDC
		int len = 0;
		//uint8_t rbyte;
	#endif

	if(mySerial.available()) {
		//isWork = true;

		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] softwareSerialEvent: Reading from serial..."));
		#endif

		while (mySerial.available()) {
		
			//#ifndef DEBUG_BLDC
				bldc_interface_uart_process_byte(mySerial.read());
				#ifdef DEBUG_BLDC
					len += sizeof(uint8_t);
				#endif
			/*#else
				rbyte = mySerial.read();
				bldc_interface_uart_process_byte(rbyte);
				Serial.write(rbyte);
				len += sizeof(uint8_t);
			#endif*/
					//delay(2);
		}

		
		#ifdef DEBUG_BLDC
			Serial.print(F("[ bldc ] softwareSerialEvent: Just read "));
			Serial.print(len);
			Serial.println(F(" bytes."));
		#endif

		is_data_ready = true;
		
	}
	else {
		#ifdef DEBUG_BLDC
			Serial.println(F("[ bldc ] softwareSerialEvent: nothing to do... "));
		#endif
	}

	
}

/**
 * Callback that the packet handler uses to send an assembled packet.
 *
 * @param data
 * Data array pointer
 * @param len
 * Data array length
 */
void send_packet(unsigned char *data, int len) {
	/*if (len > (PACKET_MAX_PL_LEN + 5)) {
		return;
	}*/
	//data[0] = 4;
	// Send the data over UART
	#ifdef DEBUG_BLDC
		Serial.print(F("[ bldc ] send_packet = len["));
		Serial.print(len);
		Serial.print(F("] write["));
		for (int i = 0; i < len ; i++){
			Serial.print(data[i]);
		Serial.print(F(":"));
		}
		//Serial.write(data, len);
		Serial.println(F("]"));
	#endif
	
	mySerial.write(data, len);
}

 void comm_uart_arduino_init() {
	// Initialize UART
	mySerial.begin(__CUA_UART_BAUDRATE);
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

	/*set timer1 interrupt at 1Hz
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
	TIMSK1 |= (1 << OCIE1A);*/

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

	// Allow interrupts so that the Processing and Timer interrupts ("Threads") start
	interrupts();
}

/*
ISR(TIMER0_COMPA_vect){//timer0 interrupt 61.1Hz 

}
*/