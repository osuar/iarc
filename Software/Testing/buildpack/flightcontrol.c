#include <stdlib.h>
#include "avr_compiler.h"
#include "support.h"
#include "usart_driver.h"
#include <stdio.h>

USART_data_t IMU;
USART_data_t xbee;

volatile char input;
volatile char readflag = 0;

int main(void){
	int i;
	int j;
	int k;

	char readcheck = 0;

	char xbeebuffer[100];
	char receive[100];
	char bytetobuffer;

	char error[10];
	sprintf(error, "error\n\r");

	char imuread = 0xD2;

	char accelcash[3] = {0,0,0};
	char rollcash[3] = {0,0,0};

	short int motorr = 0;
	short int motorl = 0;
	short int servor = 0;
	short int servol = 0;

	/**Move cmd vars*/
	short int rise = 0;
	short int rotate = 0;
	short int forward = 0;
	short int tilt = 0;

	enum states {running, stopped} state = stopped;

	PORTF.DIR = 0x03;
	PORTD.DIR = 0x0F;

	//Pulse width modulation setup for servos, port D
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 40000;

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
		if(readflag){
			readflag = 0;
			PORTF.OUT ^= 0x01;
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
			else if(input == 's'){
				state = stopped;
			}
			else if(input == 'u'){
				rise += 25;
			}
			else if(input == 'd'){
				rise -= 25;
			}
			else if(input == 'c'){
				rotate += 10;
			}
			else if(input == 'x'){
				rotate -= 10;
			}
			else if(input == 'a'){
				tilt += 10;
			}
			else if(input == 'e'){
				tilt -= 10;
			}
			else if(input == 't'){
				forward += 10;
			}
			else if(input == 'b'){
				forward -= 10;
			}
		}

		switch(state){
			case stopped:
				TCD0.CCA = 2000;
				TCD0.CCB = SERVOLINI;
				TCD0.CCC = 2000;
				TCD0.CCD = SERVORINI;
				break;
			case running:
				if(TCD0.INTFLAGS & 0x01){
					TCD0.INTFLAGS = 0x01;
					do{
						bytetobuffer = USART_TXBuffer_PutByte(&IMU, imuread);
					}while(!bytetobuffer);

					k = 0;
					while(k < 1000){
						if(USART_RXBufferData_Available(&IMU)){
							readcheck = 1;
							break;
						}
						k ++;
					}
					if(!readcheck){
						for(i = 0; error[i] != 0; i ++){
							bytetobuffer = 0;
							while(!bytetobuffer){
								bytetobuffer = USART_TXBuffer_PutByte(&xbee, error[i]);
							}
						}
					}
					readcheck = 0;

					receive[42] = USART_RXBuffer_GetByte(&IMU);		
					for(j = 1; j < 43; j ++){
						k = 0;
						while(k < 10000){
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
						accelcash[j] += output(&receive[38 - (4 * j )], 0b01111000);
						rollcash[j] += output(&receive[26 - (4 * j)], 0b01111000);

					}

					rollcash[0] -= RXN;
					rollcash[1] -= RYN;
					rollcash[2] -= RZN;
					accelcash[0] -= AXN;
					accelcash[1] -= AYN;
					accelcash[2] -= AZN;

					for(i = 0; i < 3; i ++){
						accelcash[i] /= DAMPENACCEL;
						rollcash[i] /= DAMPENROLL;
					}


					//motor updates


					ValueFunk(accelcash[0],accelcash[1],accelcash[2],rollcash[0],rollcash[1],rollcash[2],&servol,&servor,&motorl,&motorr);
					while(TCD0.CNT < 4000);

					TCD0.CCA = motorl + rise - tilt;
					TCD0.CCB = servol + rotate + forward;
					TCD0.CCC = motorr + rise + tilt;
					TCD0.CCD = servor - rotate + forward;

					sprintf(xbeebuffer, " X%4d Y%4d Z%4d x%4d y%4d z%4d R%4d r%4d L%4d l%4d\n\r", rollcash[0], rollcash[1], rollcash[2], accelcash[0], accelcash[1], accelcash[2], motorr, servor, motorl, servol);
					for(i = 0; xbeebuffer[i] != 0; i ++){
						bytetobuffer = 0;
						while(!bytetobuffer){
							bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
						}
					}
					for(i = 0; i < 3; i ++){
						accelcash[i] *= INTEGRATEACCEL;
						rollcash[i] *= INTEGRATEROLL;
					}


				}
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
	input = USART_RXBuffer_GetByte(&xbee);
	if(input == 'q'){
		CCPWrite( &RST.CTRL, 1 );
	}
	readflag = 1;
}

ISR(USARTC0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
