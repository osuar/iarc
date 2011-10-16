/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA polled USART driver example source.
 *
 *      This file contains an example application that demonstrates the polled
 *      USART driver. The code example sends all values between 0 and 255 and
 *      checks that the values received are equal to the values sent. It can be
 *      tested, using a loop-back wire between I/O pins PC2 and PC3.
 *
 * \par Application note:
 *      AVR1307: Using the XMEGA USART
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 1694 $
 * $Date: 2008-07-29 14:21:58 +0200 (ti, 29 jul 2008) $  \n
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "usart_driver.h"
#include "avr_compiler.h"


/*! Define that selects the Usart used in example. */
#define USART USARTC0


/*! Success variable, used to test driver. */
bool success;


/*! \brief Example application.
 *
 *  Example applicaton. This example configures USARTC0 for with the parameters:
 *      - 8 bit character size
 *      - No parity
 *      - 1 stop bit
 *      - 9600 Baud
 *
 *  This function then sends the values 0-255 and tests if the received data is
 *  equal to the sent data. The code can be tested by connecting PC3 to PC2. If
 *  the variable 'success' is true at the end of the function, the values
 *  have been successfully sent and received.
 */
int main(void)
{

	/* Variable used to send and receive data. */
	uint8_t sendData;
	uint8_t receivedData;

	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXD0) as output. */
	PORTC.DIRSET = PIN3_bm;

	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR = PIN2_bm;

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock fequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USART);
	USART_Tx_Enable(&USART);


	/* Assume that everything is OK. */
	success = true;
	/* Send data from 255 down to 0*/
	sendData = 255;
	while(sendData) {
	    /* Send one char. */
		do{
		/* Wait until it is possible to put data into TX data register.
		 * NOTE: If TXDataRegister never becomes empty this will be a DEADLOCK. */
		}while(!USART_IsTXDataRegisterEmpty(&USART));
		USART_PutChar(&USART, sendData);

		uint16_t timeout = 1000;
		/* Receive one char. */
		do{
		/* Wait until data received or a timeout.*/
		timeout--;
		}while(!USART_IsRXComplete(&USART) && timeout!=0);
		receivedData = USART_GetChar(&USART);

		/* Check the received data. */
		if (receivedData != sendData){
			success = false;
		}
		sendData--;
	}

	/* Disable both RX and TX. */
	USART_Rx_Disable(&USART);
	USART_Tx_Disable(&USART);

	/* If success the program ends up inside the if statment.*/
	if(success){
		while(true);
	}else{
	  	while(true);
	}
}
