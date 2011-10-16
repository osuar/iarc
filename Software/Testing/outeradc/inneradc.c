#include <stdlib.h>
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "usart_driver.h"
#include "support.h"
#include <stdio.h>
#include <stddef.h>

USART_data_t xbee;
volatile char input;
volatile char readdata = 0;

int main(void){
	unsigned int irinput;
	char xbeebuffer[100];
	PORTA.DIR = 0; 
	PORTB.DIR = 0b00000101;

	ADCA.CALL = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
	ADCA.CALH = ReadCalibrationByte( offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );	
	
	ADCA.CTRLA |= 0x01;
	ADCA.CTRLB |= 0b00000000;
	ADCA.REFCTRL = 0b00100000;
	ADCA.PRESCALER = 0b00000111;
	ADCA.CH0.MUXCTRL = 0b00010001;
	ADCA.CH0.CTRL = 0b00000010;
	

	/**Setup Xbee*/
	PORTE.DIR = 0b00001000;
	PORTF.DIR = 3;

	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();



	
	USART_InterruptDriver_Initialize(&xbee, &USARTE0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);


	while(1){
		PORTF.OUT = 0x01;

		if(input == 'r'){
			ADCA.CH0.CTRL |= 0x80;
			while(!ADCA.CH0.INTFLAGS);	
			ADCA.CH0.INTFLAGS = 0x01;
			_delay_ms(500);
			//irinput = ADCA.CH0.RESL + (256 * ADCA.CH0.RESH);
			irinput = ADCA.CH0RES;
			sprintf(xbeebuffer, " %d\n\r", irinput);
			sendstring(&xbee, xbeebuffer);
		}


	}

	return 0;
}
	
				
ISR(USARTE0_RXC_vect){
	PORTF.OUT = 1;
	USART_RXComplete(&xbee);
	input = USART_RXBuffer_GetByte(&xbee);
	readdata = 1;
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
