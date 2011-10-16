#include <stdlib.h>
#include "avr_compiler.h"
#include "usart_driver.h"
#include "twi_master_driver.h"
#include "support.h"
#include <stdio.h>

USART_data_t xbee;
TWI_Master_t imu;
volatile char readdata = 0;
volatile char input;

int main(void){

	enum states{running, stopped} state = stopped;

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

	char xbeebuffer[100];

	uint8_t accelsetupbuffer1[3] = {0x2C, 0b00001100, 0x08};
	uint8_t accelsetupbuffer2[3] = {0x31, 0x00};
	uint8_t accelstartbyte = 0x30;
	uint8_t rollsetupbuffer1[4] = {0x15, 0x04, 0x19, 0x11};
	uint8_t rollsetupbuffer2[] = {0x3E, 0b00000001};
	uint8_t rollstartbyte = 0x1A;
	
	char rollcash[3] = {0,0,0};
	int accelcash[3] = {0,0,0};


	//Pulse width modulation setup for servos, port D
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
	PORTE.DIR = 0b00001000;
	PORTF.DIR = 3;
	
	USART_InterruptDriver_Initialize(&xbee, &USARTE0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);
	

	while(1){
		if(readdata){
			readdata = 0;
			sendchar(&xbee, input);
			if(input == 'r'){
				state = running;
			}
			else if(input == 's'){
				state = stopped;
				sprintf(xbeebuffer, "rise %4d tilt %4d rot %4d for %4d \n\r", rise, tilt, rotate, forward);
				sendstring(&xbee, xbeebuffer);
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
				TCD1.CCA = 2000;
				TCC1.CCA = SERVOLINI;
				TCD1.CCB = 2000;
				TCC1.CCB = SERVORINI;
				break;

			case running:		

				if(TCC0.INTFLAGS & 0x01){
					TCC0.INTFLAGS = 0x01;
					do{
						while(imu.status != TWIM_STATUS_READY);
						TWI_MasterWriteRead(&imu, ROLL, &rollstartbyte, 1, 10);
						PORTF.OUT = 2;
						while(imu.result == TWIM_RESULT_UNKNOWN);
					}while(!(imu.readData[0] & 0x01));
					for(i = 0; i < 5; i += 2){
						rollcash[i/2] += ((char)(imu.readData[i + 3]));
					}
					do{
						while(imu.status != TWIM_STATUS_READY);
						TWI_MasterWriteRead(&imu, ACCEL, &accelstartbyte, 1, 10);
						PORTF.OUT = 3;
						while(imu.result == TWIM_RESULT_UNKNOWN);
						PORTF.OUT = 1;
					}while(!(imu.readData[0] & 0x80));

					for(i = 0; i < 5; i += 2){
						if(imu.readData[i + 3] & 0x80){
							accelcash[i/2] -= 256 * (~imu.readData[i + 3] + 1);
							accelcash[i/2] -= ~imu.readData[i + 2] + 1;
						}
						else{
							accelcash[i/2] += 256 * imu.readData[i + 3];
							accelcash[i/2] += imu.readData[i + 2];
						}
					}
					PORTF.OUT = 0;

				}

				for(i = 0; i < 3; i ++){
					accelcash[i] /= DAMPENACCEL;
					rollcash[i] /= DAMPENROLL;
				}

				ValueFunk(accelcash[0],accelcash[1],accelcash[2],rollcash[0],rollcash[1],rollcash[2],&servol,&servor,&motorl,&motorr);
				while(TCD1.CNT < 4000);

				TCD1.CCA = motorl + rise - tilt;
				TCD1.CCB = motorr + rise + tilt;
				/*

				while(TCC1.CNT < 4000);

				TCC1.CCA = servol + rotate + forward;
				TCC1.CCB = servor - rotate + forward;
				*/

				sprintf(xbeebuffer, " X%4d x%4d R%4d L%4d\n\r", rollcash[1], accelcash[0],motorr, motorl);
				sendstring(&xbee, xbeebuffer);
				
				for(i = 0; i < 3; i ++){
					accelcash[i] *= INTEGRATEACCEL;
					rollcash[i] *= INTEGRATEROLL;
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
