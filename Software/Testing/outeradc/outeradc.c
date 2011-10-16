#include <stdlib.h>
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "usart_driver.h"
#include "support.h"
#include <stdio.h>

USART_data_t xbee;
volatile char input;
volatile char readdata = 0;

int main(void){
	char irinput;
	char xbeebuffer[100];
	PORTA.DIR = 0; 
	PORTB.DIR = 0b00000101;

	/**Setup Xbee*/
	PORTE.DIR = 0b00001000;
	PORTF.DIR = 3;
	
	USART_InterruptDriver_Initialize(&xbee, &USARTE0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);

	while(1){
		if(readdata){
			readdata = 0;
			sendchar(&xbee, &input);
			if(input == 'r'){
				PORTB.OUT |= 0x01;
				_delay_ms(1);
				while(PORTB.IN & 0x02);
				PORTB.OUT &= ~0x01;
				PORTB.OUT &= ~0x04;
				_delay_ms(1);
				PORTB.OUT |= 0x04;
				irinput = PORTA.IN;
				sprintf(xbeebuffer, " %d\n\r", irinput);
				sendstring(&xbee, xbeebuffer);
			}
		}
	}
}
	
				
ISR(USARTE0_RXC_vect){
	USART_RXComplete(&xbee);
	input = USART_RXBuffer_GetByte(&xbee);
	readdata = 1;
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
