#include <stdlib.h>
#include "../../drivers/avr_compiler.h"
#include "../../drivers/usart_driver.h"
#include "../../drivers/twi_master_driver.h"
#include "support.h"
#include <stdio.h>

USART_data_t xbee;

enum states{running, stopped, offset} state = stopped;
volatile char readdata = 0;

TWI_Master_t imu;
volatile char input[9] = {0,0,0,0,0,0,0,0,0};
volatile char xbeecounter = 0;

int main(void){
	int i;
	short int motor1;
	short int motor2;
	short int motor3;
	short int motor4;

	uint8_t accelstartbyte = 0x30;
	uint8_t gyrostartbyte = 0x1A;

	char joyaxis[] = {0,0,0,0};

	int motorup = 0;

	char gyrocache[3] = {0,0,0};
	int accelcache[3] = {0,0,0};
	int accelint[] = {0, 0, 0};
	int gyroint[] = {0, 0, 0};

	char rolhisx[50];
	char rolhisy[50];
	char rolhisz[50];
	int acchisx[50];
	int acchisy[50];
	int acchisz[50];

	char readyset = 0;

	int accelnorm[3] = {0,0,0};
	char gyronorm[3] = {0,0,0};


	char xbeebuffer[100];

	PORTD.DIR = 0x0F;
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCCEN_bm |  TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 40000;

	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 40000;

	PORTE.DIR = 0x08;
	PORTF.DIR = 0x03;
	PORTC.DIR = 0b00001100;
	PORTC.OUT = 0b00001000;

	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
		PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;

		TCD0.CCA = 2000;
		TCD0.CCB = 2000;
		TCD0.CCC = 2000;
		TCD0.CCD = 2000;


	sei();
	uartInitiate(&xbee, &USARTE0);
	twiInitiate(&imu, &TWIC);
	sprintf(xbeebuffer, "starting\n");
	sendstring(&xbee, xbeebuffer);

	while(1){
		if(readdata){

			readdata = 0;
			/*

			   if(input[0] == 'r'){
			   state = running;
			   sprintf(xbeebuffer, "running\r\n");
			   sendstring(&xbee, xbeebuffer);
			   }
			   else if(input[0] == 's'){
			   state =stopped;
			   sprintf(xbeebuffer, "stopped\r\n");
			   sendstring(&xbee, xbeebuffer);

			   }
			 */


			for(i = 0; i < 4; i ++){
				joyaxis[i] = input[3 + i] - 126;
			}

			if(input[7] == 3){
				state = stopped;
				sprintf(xbeebuffer, "stopped %d\n", input[7]);
				//sprintf(xbeebuffer, "%4d %4d %4d %4d\n\r", motor1, motor2, motor3, motor4);
				sendstring(&xbee, xbeebuffer);
			}
			else if(input[7] == 4){
				state = running;
				sprintf(xbeebuffer, "running %d\n", input[7]);
				sendstring(&xbee, xbeebuffer);
			}
			else if(input[7] == 5){
				state = offset;
				sprintf(xbeebuffer, "offsetting\n");
				sendstring(&xbee, xbeebuffer);
			}
			else if(input[7] == 6){
				motorup += 25;
			}
			else if(input[7] == 7){
				motorup -= 25;
			}
			xbeecounter = 0;


		}

		switch(state){
			case stopped:
				TCD0.CCA = 2000;
				TCD0.CCB = 2000;
				TCD0.CCC = 2000;
				TCD0.CCD = 2000;
				break;
			case offset:
				updateoffset(&imu,
						accelnorm,
						gyronorm,
						rolhisx,
						rolhisy,
						rolhisz,
						acchisx,
						acchisy,
						acchisz,
						accelcache,
						gyrocache, 
						&readyset, 
						&gyrostartbyte, 
						&accelstartbyte);
				state = stopped;

				break;



			case running:
				while(!(TCC0.INTFLAGS & 0x01));

				TCC0.INTFLAGS = 0x01;

				getgyro(gyrocache, &imu, &gyrostartbyte);

				for(i = 0; i < 3; i ++){
					gyrocache[i] -= gyronorm[i];
				}

				for(i = 0; i < 3; i ++){
					gyroint[i] = ((10 * gyroint[i]) + (10 * gyrocache[i]))/20;
				}
				
				getaccel(accelcache, &imu, &accelstartbyte);
				for(i = 0; i < 3; i ++){
					accelcache[i] -= accelnorm[i];
				}

				for(i = 0; i < 3; i ++){
					accelint[i] = ((16 * accelint[i]) + (4 * accelcache[i]))/20;
				}

				ValueFunk(accelint[0], accelint[1], accelint[2], gyroint[0], gyroint[1], gyroint[2], &motor2, &motor4, &motor1, &motor3);

				//sprintf(xbeebuffer, "%4d %4d %4d %4d\n\r", motor1, motor2, motor3, motor4);
				//sprintf(xbeebuffer, "X-%3d Y-%3d Z-%3d x-%3d y-%3d z-%3d\n\r", gyroint[0], gyroint[1], gyroint[2], accelint[0], accelint[1], accelint[2]);
				//sendstring(&xbee, xbeebuffer);


				while(TCD0.CNT < 4000);
				TCD0.CCA = motor1 - (joyaxis[2] * 2) + joyaxis[0] -(2 * joyaxis[3]) + motorup;
				TCD0.CCC = motor3 - (joyaxis[2] * 2) - joyaxis[0] -(2 * joyaxis[3]) + motorup;
				TCD0.CCB = motor2 - (joyaxis[2] * 2)- (1 * joyaxis[1]) + (2 * joyaxis[3]) + motorup;
				TCD0.CCD = motor4 - (joyaxis[2] * 2) + (1 * joyaxis[1]) + (2* joyaxis[3]) + motorup;

				for(i = 0; i < 3; i ++){
					accelcache[i] = 0;
					gyrocache[i] = 0;
				}

				break;
		}
	}
}


ISR(USARTE0_RXC_vect){
	USART_RXComplete(&xbee);
	input[xbeecounter] = USART_RXBuffer_GetByte(&xbee);


	if((input[0] == ' ') && (xbeecounter == 0)){
		xbeecounter ++;
	}
	else if((input[1] == 's') && (xbeecounter == 1)){
		xbeecounter ++;
	}
	else if((input[2] == 'a') && (xbeecounter == 2)){
		xbeecounter ++;
	}
	else if((xbeecounter >= 3) && (xbeecounter <= 7)){
		xbeecounter ++;
	}
	else if((input[8] == 'r') && (xbeecounter == 8)){
		readdata = 1;
		PORTF.OUT ^= 0x01;
		if(input[7] == 1){
			CCPWrite( &RST.CTRL, 1 );
		}
		xbeecounter ++;
	}



}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&imu);
}

