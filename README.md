This is port for the Arduino Pro Mini board of the test project for UART communication between the VESC and the stm32discovery board. Almost the full interface of the VESC is implemented in the example, only firmware upload is missing.  
  
Connect TX to RX on the VESC  
Connect RX to TX on the VESC  
Connect the grounds between the arduino board and the VESC.  

To upload the project to an arduino board, just use the Arduino IDE and include the following lines at the top of your sketch to use it:  

`#include <datatypes.h>`
`#include <comm_uart_arduino.h>`
`#include <bldc_interface.h>`

Further documentation and example sketches will be included if there is interest from the community.  
  
Vedder has written a tutorial on how to use and port his code to other platforms here:  
http://vedder.se/2015/10/communicating-with-the-vesc-using-uart/

I based my work on his and the information provided was enough to get me started. Drop me a line if you need help and I'll update the project for all.
