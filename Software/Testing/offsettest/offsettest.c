#include <stdlib.h>
#include "avr_compiler.h"
#include "usart_driver.h"
#include "twi_master_driver.h"
#include "support.h"
#include <stdio.h>

USART_data_t xbee;
//USART_data_t mega;
TWI_Master_t imu;
volatile char readdata = 0;
//volatile char readirdata = 0;
volatile char input;
//volatile char inputir;

int main(void){

	enum states{running, stopped} state = stopped;

	int i;
	int j;

	char xbeebuffer[100];

	uint8_t accelsetupbuffer1[3] = {0x2C, 0b00001001, 0x08};
	uint8_t accelsetupbuffer2[3] = {0x31, 0x00};
	uint8_t accelstartbyte = 0x30;
	uint8_t rollsetupbuffer1[4] = {0x15, 20, 0b00011011, 0x11};
	uint8_t rollsetupbuffer2[] = {0x3E, 0b00000001};
	uint8_t rollstartbyte = 0x1A;
	//char getirbyte = 'r';
	
	char rollcash[3] = {0,0,0};
	int rollint[] = {0, 0, 0};
	int accelcash[3] = {0,0,0};
	int accelint[] = {0, 0, 0};
	
	char rolhisx[50];
	char rolhisy[50];
	char rolhisz[50];
	int acchisx[50];
	int acchisy[50];
	int acchisz[50];

	char readyset = 0;
	char offsetgood = 0;
	char offsetprevious = 0;

	int accelnorm[3] = {0,0,0};
	char rollnorm[3] = {0,0,0};

	int runningroll[3];
	int runningaccel[3];
	

	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 40000;



	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();


	//Setup IMU
	PORTD.DIR = 0x30;
	PORTC.DIR = 0b00111100;
	PORTC.OUT = 0b00001000;

	TWI_MasterInit(&imu, &TWIC, TWI_MASTER_INTLVL_HI_gc, TWI_BAUDSETTING);

	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ACCEL, accelsetupbuffer1, 3, 0);
	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ACCEL, accelsetupbuffer2, 2, 0);
	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer1, 4, 0);
	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer2, 2, 0);
	while(imu.status != TWIM_STATUS_READY);
	TWIC.MASTER.CTRLB |= 0x0C;


	/**Setup Xbee*/
	PORTE.DIR |= 0b00001000;
	PORTF.DIR |= 3;
	PORTF.OUT = 1;
	
	USART_InterruptDriver_Initialize(&xbee, &USARTE0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);

	while(1){
		for(i = 0; i < 3; i ++){
			rollcash[i] = 0;
			accelcash[i] = 0;
			runningroll[i] = 0;
			runningaccel[i] = 0;
		}
		if(readdata){
			readdata = 0;
			PORTF.OUT ^= 0x02;
			sendchar(&xbee, input);
			if(input == 'r'){
				state = running;
			}
			else if(input == 's'){
				PORTF.OUT ^= 0x02;
				state = stopped;
				//sprintf(xbeebuffer, "rise %4d tilt %4d rot %4d for %4d \n\r", rise, tilt, rotate, forward);
				sprintf(xbeebuffer, "X %d Y %d Z %d x %d y %d z %d\n\r", rollnorm[0], rollnorm[1], rollnorm[2], accelnorm[0], accelnorm[1], accelnorm[2]);
				sendstring(&xbee, xbeebuffer);
			}
		}

		switch(state){
			case stopped:
				if(!readyset){
					for(i = 0; i < 50; i ++){
						for(j = 0; j < 3; j ++){
							rollcash[j] = 0;
							accelcash[j] = 0;
						}
						while(!(TCC0.INTFLAGS & 0x01));
						TCC0.INTFLAGS |= 0x01;
						getroll(rollcash, &imu, &rollstartbyte);
						getaccel(accelcash, &imu, &accelstartbyte);
						sprintf(xbeebuffer ," X %d Y %d Z %d x %d y %d z %d\n\r", rollcash[0], rollcash[1], rollcash[2], accelcash[0], accelcash[1], accelcash[2]);
						sendstring(&xbee, xbeebuffer);
						
						acchisx[i] = accelcash[0];
						acchisy[i] = accelcash[1];
						acchisz[i] = accelcash[2];
						rolhisx[i] = rollcash[0];
						rolhisy[i] = rollcash[1];
						rolhisz[i] = rollcash[2];
					}
					readyset = 1;
				}
				
				else{
					PORTF.OUT = 2;
					for(i = 49; i > 0; i --){
						acchisx[i] = acchisx[i - 1];
						acchisy[i] = acchisy[i - 1];
						acchisz[i] = acchisz[i - 1];
						rolhisx[i] = rolhisx[i - 1];
						rolhisy[i] = rolhisy[i - 1];
						rolhisz[i] = rolhisz[i - 1];
					}
					while(!(TCC0.INTFLAGS & 0x01));
					getroll(rollcash, &imu, &rollstartbyte);
					getaccel(accelcash, &imu, &accelstartbyte);
					acchisx[0] = accelcash[0];
					acchisy[0] = accelcash[1];
					acchisz[0] = accelcash[2];
					rolhisx[0] = rollcash[0];
					rolhisy[0] = rollcash[1];
					rolhisz[0] = rollcash[2];
					
				//	sprintf(xbeebuffer, "X0 %d X1 %d X2 %d\n\r", acchisx[0], acchisx[1], acchisx[2]);
				//	sendstring(&xbee, xbeebuffer);
				}
				if(TCC0.INTFLAGS & 0x01){
					offsetgood = getoffset(acchisx, acchisy, acchisz, rolhisx, rolhisy, rolhisz, accelnorm, rollnorm);
					if(offsetgood){
						if(offsetprevious != offsetgood){
							offsetprevious = offsetgood;
							sprintf(xbeebuffer, "ready\n\r");
							sendstring(&xbee, xbeebuffer);
						}
					}
					else{
						if(offsetprevious != offsetgood){
							offsetprevious = offsetgood;
							sprintf(xbeebuffer, "ready\n\r");
							sendstring(&xbee, xbeebuffer);
						}
					}
				}
				 

				break;

			case running:	

				for(i = 0; i < 1; i ++){
					if(TCC0.INTFLAGS & 0x01){

						TCC0.INTFLAGS = 0x01;

						getroll(rollcash, &imu, &rollstartbyte);
						getaccel(accelcash, &imu, &accelstartbyte);
						for(j = 0; j < 3; j ++){
							runningroll[j] += rollcash[j];
							runningaccel[j] += accelcash[j];
						}
					}
				}
				for(i = 0; i < 3; i++){
					rollcash[i] = runningroll[i] / 1;
					accelcash[i] = runningaccel[i] / 1;
				}

				for(i = 0; i < 3; i ++){
					accelcash[i] -= accelnorm[i];
					rollcash[i] -= rollnorm[i];
				}


				for(i = 0; i < 3; i ++){
					accelint[i] = ((19 * accelint[i]) + accelcash[i])/20;
					rollint[i] = ((15 * rollint[i]) + (5 * rollcash[i]))/20;
				}



				//sprintf(xbeebuffer, " X%4d x%4d R%4d L%4d\n\r", rollint[1], accelint[0],motorr, motorl);
				sprintf(xbeebuffer, " X %d Y %d Z %d x %d y %d z %d\n\r", rollint[0], rollint[1], rollint[2], accelint[0], accelint[1], accelint[2]);
				sendstring(&xbee, xbeebuffer);

				for(i = 0; i < 3; i ++){
					accelcash[i] = 0;
					rollcash[i] = 0;
				}
				break;
		}
	}





	return 0;
}
ISR(USARTE0_RXC_vect){
	USART_RXComplete(&xbee);
	input = USART_RXBuffer_GetByte(&xbee);
	if(input == 'q'){
		CCPWrite( &RST.CTRL, 1 );
	}
	readdata = 1;
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&imu);
}
