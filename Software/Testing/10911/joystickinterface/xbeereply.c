#include <stdlib.h>
#include <stdio.h>
#include "support.h"
#include "../../drivers/avr_compiler.h"
#include "../../drivers/usart_driver.h"

USART_data_t xbee;
volatile char readdata = 0;
volatile char input;

int main(void){
	
	PORTD.DIR = 0x08;
	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	uartInitiate(&xbee, &USARTD0);

	while(1){
		if(readdata){
			readdata = 0;
			senchar(&xbee, input);
		}
	}
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
