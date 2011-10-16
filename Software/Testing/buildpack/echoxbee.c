#include <stdlib.h>
#include "avr_compiler.h"
#include "support.h"
#include "usart_driver.h"
#include <stdio.h>

USART_data_t xbee;

int main(void){
	char input;

	int i;

	char xbeebuffer[100];
	char bytetobuffer;


	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	/**Setup Xbee*/
	PORTC.DIR = 0b00001000;
	
	USART_InterruptDriver_Initialize(&xbee, &USARTC0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTC0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);
	

	while(1){
		if(USART_RXBufferData_Available(&xbee)){
			PORTF.OUT |= 0x01;
			input = USART_RXBuffer_GetByte(&xbee);
			sprintf(xbeebuffer, "%c", input);
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}	
			}
		}
	}
	return 0;
}
ISR(USARTC0_RXC_vect){
	USART_RXComplete(&xbee);
}

ISR(USARTC0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
