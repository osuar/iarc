#include <stdlib.h>
#include "../Software/Testing/drivers/avr_compiler.h"
#include "../Software/Testing/drivers/usart_driver.h"
#include "../Software/Testing/drivers/twi_master_driver.h"
#include "support.h"
#include <stdio.h>

USART_data_t xbee;
USART_data_t mega;
//USART_data_t mega;
TWI_Master_t imu;
volatile char readdata = 0;
volatile char readirdata = 0;
volatile char input[9] = {0,0,0,0,0,0,0,0,0};
volatile char inputir[4];
volatile char ir = 0;
volatile char xbeecounter = 0;

enum states{running, stopped, offset} state = stopped;

int main(void){


	//acceleration flags
	short int auf = 0;
	short int adf = 0;
	short int abf = 0;
	short int aff = 0;
	
	char goingup = 0;
	char goingdown = 0;
	char goingforward = 0;
	char goingback = 0;

	char accelupward = 0;
	char accelforward = 0;

	char joyaxis[] = {0,0,0,0};

	short int motorr = 0;
	short int motorl = 0;
	short int servor = 0;
	short int servol = 0;

	int i;
	int m = 0;

	char xbeebuffer[100];

	uint8_t accelstartbyte = 0x30;
	uint8_t rollstartbyte = 0x1A;

	int megatimeout = 0;

	char getirbyte = 'r';
	
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

	int accelnorm[3] = {0,0,0};
	char rollnorm[3] = {0,0,0};

	char accelflag = 1;

//Setup IO
	PORTD.DIR = 0x38;
	PORTC.DIR = 0b00111100;
	PORTC.OUT = 0b00001000;
	PORTE.DIR |= 0b00001000;
	PORTF.DIR |= 3;

	TCD1.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD1.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm |TC0_CCBEN_bm;
	TCD1.PER = 40000;

	TCC1.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC1.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm |TC0_CCBEN_bm;
	TCC1.PER = 40000;

	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 40000;

	TCD1.CCA = 2000;
	TCD1.CCB = 2000;



	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	uartInitiate(&xbee, &USARTD0);
	uartInitiate(&mega, &USARTE0);
	twiInitiate(&imu, &TWIC);
	sprintf(xbeebuffer, "Starting\n");
	sendstring(&xbee, xbeebuffer);

	while(1){
		PORTF.OUT = 0;

		
		for(i = 0; i < 3; i ++){
			rollcash[i] = 0;
			accelcash[i] = 0;
		}
		
		if(readdata){
			//sendchar(&xbee, input[0]);
			//sprintf(xbeebuffer, "%c %c %c %d %d %d %d %d %c\n\r", input[0], input[1], input[2], input[3], input[4], input[5], input[6], input[7], input[8]);
			//sendstring(&xbee, xbeebuffer);
			readdata = 0;
			for(i = 0; i < 4; i ++){
				joyaxis[i] = input[3 + i] - 126;
			}
			
			if(input[7] == 3){
				state = stopped;
				sprintf(xbeebuffer, "stopped\n");
				sendstring(&xbee, xbeebuffer);
			}
			else if(input[7] == 4){
				state = running;
				sprintf(xbeebuffer, "running\n");
				sendstring(&xbee, xbeebuffer);
			}
			else if(input[7] == 5){
				state = offset;
				sprintf(xbeebuffer, "offsetting\n");
				sendstring(&xbee, xbeebuffer);
			}
			
			xbeecounter = 0;
		
		}
				

			switch(state){
			case stopped:
			TCD1.CCA = 2000;
			TCC1.CCA = SERVOLINI;
			TCD1.CCB = 2000;
			TCC1.CCB = SERVORINI;
			break;

			case offset:
			updateoffset(&imu,
			accelnorm,
			rollnorm,
			rolhisx,
			rolhisy,
			rolhisz,
			acchisx,
			acchisy,
			acchisz,
			accelcash,
			rollcash, 
			&readyset, 
			&rollstartbyte, 
			&accelstartbyte);
			state = stopped;

			break;


			case running:	

			while(!(TCC0.INTFLAGS & 0x01));

			TCC0.INTFLAGS = 0x01;

			getroll(rollcash, &imu, &rollstartbyte);
			if(accelflag == 1){
				accelflag = 0;
				getaccel(accelcash, &imu, &accelstartbyte);
			}
			else{
				accelflag = 1;
				for(i = 0; i < 3; i ++){
					accelcash[i] = accelint[i] + accelnorm[i];
				}
			}

			if(m < 5){
				m ++;
			}
			else{
				m = 0;
				sendchar(&mega, getirbyte);
			}

			for(i = 0; i < 3; i ++){
				accelcash[i] -= accelnorm[i];
				rollcash[i] -= rollnorm[i];
			}


			for(i = 0; i < 3; i ++){
				accelint[i] = ((18 * accelint[i]) + (2 * accelcash[i]))/20;
				rollint[i] = ((18 * rollint[i]) + (2 * rollcash[i]))/20;
			}



			ValueFunk(accelint[0],accelint[1],accelint[2],rollint[0],rollint[1],rollint[2],&servol,&servor,&motorl,&motorr);


			if(m == 0){

				while((!readirdata) && (megatimeout > 10000)){
					PORTF.OUT = 0x01;
					megatimeout ++;
					if(megatimeout > 9999){
						ir = 0;
					}
				}


				megatimeout = 0;
				readirdata = 0;

				//sprintf(xbeebuffer, " %3d %3d\n\r", inputir[2],inputir[3]);
			//	sprintf(xbeebuffer, "ir\n\r");
			//	sendstring(&xbee, xbeebuffer);

				//if too close backup
				if((inputir[2] > HORIZUPPER) && !goingback){
					abf = 1; //accelbackflag
					goingback = 1;
					sprintf(xbeebuffer, "Too Close\n\r");
					sendstring(&xbee, xbeebuffer);
				}
				//if too far move forward
				else if((inputir[2] < HORIZLOWER) && !goingforward){
					aff = 1; //accelforwardflag
					goingforward = 1;
					sprintf(xbeebuffer, "Too Far\n\r");
					sendstring(&xbee, xbeebuffer);
				}

				//if too low raise
				if((inputir[3] > VERTUPPER) && (!goingup)){
					auf = 1; //accelupflag
					goingup = 1;
					sprintf(xbeebuffer, "Too Low\n\r");
					sendstring(&xbee, xbeebuffer);
				}
				else if((inputir[3] < VERTLOWER) && (!goingdown)){
					adf = 1; //acceldownflag
					goingdown = 1;
					sprintf(xbeebuffer, "Too High\n\r");
					sendstring(&xbee, xbeebuffer);
				}

				//if returned accel to stop
				sprintf(xbeebuffer, "Good Horiz\n\r");
				if(goingback && (inputir[2] < HORIZUPPER)){
					goingback = 0;
					aff = HORIZAF / 2;
					sendstring(&xbee, xbeebuffer);
				}
				else if(goingforward && (inputir[2] > HORIZLOWER)){
					goingforward = 0;
					abf = HORIZAF / 2;
					sendstring(&xbee, xbeebuffer);
				}

				sprintf(xbeebuffer, "Good Vert\n\r");
				if(goingup && (inputir[3] < VERTUPPER)){
					goingup = 0;
					adf = 1;
					sendstring(&xbee, xbeebuffer);
				}
				else if(goingdown && (inputir[3] > VERTLOWER)){
					goingdown = 0;
					auf = 1;
					sendstring(&xbee, xbeebuffer);
				}

			}


			if((aff) && (aff < HORIZVEL)){
				aff++;
				accelforward = HORIZAF;
			}
			else{
				aff = 0;
			}

			if((abf) && (abf < HORIZVEL)){
				abf ++;
				accelforward = -HORIZAF;
			}
			else{
				abf = 0;
			}

			if((auf) && (auf < VERTVEL)){
				auf ++;
				accelupward = VERTAU;
			}
			else{
				auf = 0;
			}

			if((adf) && (adf < VERTVEL)){
				adf ++;
				accelupward = -VERTAU;
			}
			else{
				adf = 0;
			}

			if(!adf && !auf){
				accelupward = 0;
			}
			if(!abf && !aff){
				accelforward = 0;
			}



			while(TCD1.CNT < 4000);

			TCD1.CCA = motorl - (joyaxis[2] * 2) + joyaxis[0] + accelupward;
			TCD1.CCB = motorr - (joyaxis[2] * 2) - joyaxis[0] + accelupward;

			while(TCC1.CNT < 4000);

			TCC1.CCA = servol + joyaxis[3] - (1 * joyaxis[1]) - accelforward;
			TCC1.CCB = servor + joyaxis[3] + (1 * joyaxis[1]) + accelforward;

			//sprintf(xbeebuffer, " X%4d x%4d R%4d L%4d\n\r", rollint[1], accelint[0] >> 3,motorr, motorl);
			//sprintf(xbeebuffer, " X %4d Y %4d Z %4d x %4d y %4d z %4d v %2d h %2d\n\r", rollint[0], rollint[1], rollint[2], accelint[0], accelint[1], accelint[2], inputir[2], inputir[3]);
			//sendstring(&xbee, xbeebuffer);

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
	USART_RXComplete(&mega);
	inputir[ir] = USART_RXBuffer_GetByte(&mega);
	if((ir == 0) && (inputir[0] == 'r')){
		ir ++;
	}
	else if((ir == 1) && (inputir[1] == ' ')){
		PORTF.OUT = 0x02;
		ir ++;
	}
	else if(ir > 1){
		ir ++;
	}
	if(ir == 4){
		ir = 0;
		readirdata = 1;
	}
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&mega);
}

ISR(USARTD0_RXC_vect){
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
		if(input[7] == 1){
			CCPWrite( &RST.CTRL, 1 );
		}
		xbeecounter ++;
	}
}

ISR(USARTD0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}

ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&imu);
}

