#include <stdlib.h>
#include "../../drivers/avr_compiler.h"
#include "../../drivers/usart_driver.h"
#include "../../drivers/twi_master_driver.h"
#include "support.h"
#include <stdio.h>

USART_data_t xbee;
USART_data_t mega;
//USART_data_t mega;
TWI_Master_t imu;
volatile char readdata = 0;
volatile char readirdata = 0;
volatile char input;
volatile char inputir[4];
volatile char ir = 0;

int main(void){

	enum states{running, stopped, offset} state = stopped;

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

	/**Move cmd vars*/
	short int rise = 0;
	short int rotate = 0;
	short int forward = 0;
	short int tilt = 0;

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


	TCD1.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD1.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm |TC0_CCBEN_bm;
	TCD1.PER = 40000;

	TCC1.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC1.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm |TC0_CCBEN_bm;
	TCC1.PER = 40000;

	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 40000;



	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();


	//Setup IO
	PORTD.DIR = 0x38;
	PORTC.DIR = 0b00111100;
	PORTC.OUT = 0b00001000;
	PORTE.DIR |= 0b00001000;
	PORTF.DIR |= 3;


	PORTF.OUT = 0x01;
	twiInitiate(&imu, &TWIC);
	PORTF.OUT = 0x03;
	uartInitiate(&xbee, &USARTD0);
	uartInitiate(&mega, &USARTE0);

	sprintf(xbeebuffer, "Starting");
	sendstring(&xbee, xbeebuffer);

	while(1){

		for(i = 0; i < 3; i ++){
			rollcash[i] = 0;
			accelcash[i] = 0;
		}

		if(readdata){
			readdata = 0;
			//sendchar(&xbee, input);
			if(input == 'r'){
				state = running;
			}
			if(input == 'o'){
				state = offset;
			}
			else if(input == 's'){
				state = stopped;
				sprintf(xbeebuffer, "rise %4d tilt %4d rot %4d for %4d \n\r", rise, tilt, rotate, forward);
				//sprintf(xbeebuffer, "X %d Y %d Z %d x %d y %d z %d\n\r", rollnorm[0], rollnorm[1], rollnorm[2], accelnorm[0], accelnorm[1], accelnorm[2]);
				sendstring(&xbee, xbeebuffer);
			}
/*
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
				tilt += 5;
			}
			else if(input == 'e'){
				tilt -= 5;
			}
			else if(input == 't'){
				forward += 5;
			}
			else if(input == 'b'){
				forward -= 5;
			}
*/

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
					PORTF.OUT = 0x02;
					sendchar(&mega, getirbyte);
				}

				for(i = 0; i < 3; i ++){
					accelcash[i] -= accelnorm[i];
					rollcash[i] -= rollnorm[i];
				}


				for(i = 0; i < 3; i ++){
					accelint[i] = ((15 * accelint[i]) + (5 * accelcash[i]))/20;
					rollint[i] = ((18 * rollint[i]) + (2 * rollcash[i]))/20;
				}



				ValueFunk(accelint[0],accelint[1],accelint[2],rollint[0],rollint[1],rollint[2],&servol,&servor,&motorl,&motorr);


				if(m == 0){

					while((!readirdata) || (megatimeout > 10000)){
						megatimeout ++;
						if(megatimeout > 9999){
							ir = 0;
						}
					}


					megatimeout = 0;
					readirdata = 0;

					sprintf(xbeebuffer, " %3d %3d\n\r", inputir[2],inputir[3]);
					sendstring(&xbee, xbeebuffer);

					//if too close backup
					if(inputir[2] > HORIZUPPER){
						abf = 1; //accelbackflag
						goingback = 1;
						sprintf(xbeebuffer, "Too Close\n\r");
						sendstring(&xbee, xbeebuffer);
					}
					//if too far move forward
					else if(inputir[2] < HORIZLOWER){
						aff = 1; //accelforwardflag
						goingforward = 1;
						sprintf(xbeebuffer, "Too Far\n\r");
						sendstring(&xbee, xbeebuffer);
					}

					//if too low raise
					if(inputir[3] > VERTUPPER){
						auf = 1; //accelupflag
						goingup = 1;
						sprintf(xbeebuffer, "Too Low\n\r");
						sendstring(&xbee, xbeebuffer);
					}
					else if(inputir[3] < VERTLOWER){
					PORTF.OUT = 0x01;

						adf = 1; //acceldownflag
						goingdown = 1;
						sprintf(xbeebuffer, "Too High\n\r");
					}

					//if returned accel to stop
					if(goingback && (inputir[2] < HORIZUPPER)){
						goingback = 0;
						aff = 1;
					}
					else if(goingforward && (inputir[2] > HORIZLOWER)){
						goingforward = 0;
						abf = 1;
					}

					if(goingup && (inputir[3] < VERTUPPER)){
						goingup = 0;
						adf = 1;
					}
					else if(goingdown && (inputir[3] > VERTLOWER)){
						goingdown = 0;
						auf = 1;
					}

				}


				if((aff) && (aff < HORIZVEL)){
					aff++;
					accelforward = HORIZAF;
				}
				else{
					aff = 0;
					accelforward = 0;
				}

				if((abf) && (abf < HORIZVEL)){
					abf ++;
					accelforward = -HORIZAF;
				}
				else{
					abf = 0;
					accelforward = 0;
				}

				if((auf) && (auf < VERTVEL)){
					auf ++;
					accelupward = VERTAU;
				}
				else{
					auf = 0;
					accelupward = 0;
				}

				if((adf) && (adf < VERTVEL)){
					adf ++;
					accelupward = -VERTAU;
				}
				else{
					adf = 0;
					accelupward = -VERTAU;
				}



				while(TCD1.CNT < 4000);

				TCD1.CCA = motorl + rise - tilt + accelupward;
				TCD1.CCB = motorr + rise + tilt + accelupward;

				while(TCC1.CNT < 4000);

				TCC1.CCA = servol + rotate - forward - accelforward;
				TCC1.CCB = servor + rotate + forward + accelforward;

				//sprintf(xbeebuffer, " X%4d x%4d R%4d L%4d\n\r", rollint[1], accelint[0] >> 3,motorr, motorl);
				//sprintf(xbeebuffer, " X %4d Y %4d Z %4d x %4d y %4d z %4d\n\r", rollint[0], rollint[1], rollint[2], accelint[0], accelint[1], accelint[2]);
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
	input = USART_RXBuffer_GetByte(&xbee);
	if(input == 'q'){
		CCPWrite( &RST.CTRL, 1 );
	}
	readdata = 1;
}

ISR(USARTD0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}

ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&imu);
}

