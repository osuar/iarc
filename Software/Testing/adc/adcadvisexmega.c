#include <stdlib.h>
#include "../drivers/avr_compiler.h"
#include "../drivers/usart_driver.h"
#include "../drivers/twi_master_driver.h"
#include "support.h"
#include <stdio.h>

USART_data_t xbee;
USART_data_t mega;

volatile char inputir;
volatile char input;
volatile char readdata = 0;
volatile char readdatair = 0;

int main(void){

	int i = 0;
	char getirbyte = 'r';

	PORTD.DIR = 0b00110000;

	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 40000;

	TCD1.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD1.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm | TC0_CCBEN_bm;
	TCD1.PER = 40000;

	TCD1.CCA = 2000;
	TCD1.CCB = 2000;

	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	PORTD.DIR |= 0x08;
	PORTE.DIR = 0x08;
	PORTF.DIR = 0x03;

	uartInitiate(&xbee, &USARTD0);
	uartInitiate(&mega, &USARTE0);

	while(1){


		if(TCC0.INTFLAGS & 0x01){
			TCC0.INTFLAGS = 0x01;

			i++;
			if(i > 9){
				i = 0;
				PORTF.OUT = 1;
				sendchar(&mega, getirbyte);
			}
		}

		if(readdatair){
			readdatair = 0;
			sendchar(&xbee, inputir);
		}

		if(readdata){
			readdata = 0;
			sendchar(&xbee, input);
		}
	}
	
	return 0;

}

ISR(USARTE0_RXC_vect){
	USART_RXComplete(&mega);
	inputir = USART_RXBuffer_GetByte(&mega);
	readdatair = 1;
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&mega);
}
ISR(USARTD0_RXC_vect){
	USART_RXComplete(&xbee);
	input = USART_RXBuffer_GetByte(&xbee);
	if(input == 'q'){
		CCPWrite( &RST.CTRL, 1 );
	}
	readdata = 1;
}

ISR(USARTD0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
