#include <stdlib.h>
#include "avr_compiler.h"
#include "support.h"
#include "usart_driver.h"
#include <stdio.h>

USART_data_t IMU;
USART_data_t xbee;

int main(void){
	char input;

	int i;
	int j;
	int k;

	char xbeebuffer[100];
	char receive[100];
	char bytetobuffer;

	char imuread = 5;

	char accelcash[3];
	char rollcash[3];

	enum states {running, stopped} state = stopped;

	PORTF.DIR = 0xD2;

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

	/**Setup IMU*/
	PORTE.DIR = 0b00001000;

	USART_InterruptDriver_Initialize(&IMU, &USARTE0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(IMU.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(IMU.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 19 , -4);
	USARTE0.CTRLB |= 0x04;
	USART_Rx_Enable(IMU.usart);
	USART_Tx_Enable(IMU.usart);

	while(1){
		if(USART_RXBufferData_Available(&xbee)){
			PORTF.OUT ^= 0x01;
			input = USART_RXBuffer_GetByte(&xbee);
			sprintf(xbeebuffer, "%c", input);
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}	
			}

			if(input == 'r'){
				state = running;
			}
		}

		switch(state){
			case stopped:
			
			break;
			case running:
				do{
					bytetobuffer = USART_TXBuffer_PutByte(&IMU, imuread);
				}while(!bytetobuffer);

				k = 0;
				while(1){
					if(USART_RXBufferData_Available(&IMU)){
						break;
					}
					k ++;
				}

				receive[42] = USART_RXBuffer_GetByte(&IMU);		
				for(j = 1; j < 43; j ++){
					k = 0;
					while(1){
						if(USART_RXBufferData_Available(&IMU)){
							break;
						}
						k ++;
					}
					receive[42 - j] = USART_RXBuffer_GetByte(&IMU);
				}
				PORTF.OUT |= 0x01;
				for(j = 0; j < 3; j ++){
					PORTF.OUT |= 0x02;
					accelcash[j] = output(&receive[38 - (4 * j )], 0b01111000);
					rollcash[j] = output(&receive[26 - (4 * j)], 0b01111000);

				}

				sprintf(xbeebuffer, "  %i %i %i %i %i %i\n\r", accelcash[0], accelcash[1], accelcash[2], rollcash[0], rollcash[1], rollcash[2]);	
				for(i = 0; xbeebuffer[i] != 0; i ++){
					bytetobuffer = 0;
					while(!bytetobuffer){
						bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
					}	
				}

				state = stopped;
			break;
		}

	}
	return 0;
}
ISR(USARTE0_RXC_vect){
	USART_RXComplete(&IMU);
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&IMU);
}
ISR(USARTC0_RXC_vect){
	USART_RXComplete(&xbee);
}

ISR(USARTC0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
