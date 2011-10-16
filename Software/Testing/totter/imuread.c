#include <stdlib.h>
#include "avr_compiler.h"
#include "usart_driver.h"
#include "twi_master_driver.h"
#include "support.h"
#include <stdio.h>

USART_data_t xbee;
TWI_Master_t imu;

int main(void){

	enum states{running, stopped} state = stopped;
	char input;


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

	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 40000;



	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();


	//Setup IMU
	PORTC.DIR = 0b00001100;
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
		if(USART_RXBufferData_Available(&xbee)){
			input = USART_RXBuffer_GetByte(&xbee);
			sendchar(&xbee, input);
			if(input == 'r'){
				state = running;
			}
			else if(input == 's'){
				PORTF.OUT ^= 0x02;
				state = stopped;
			}
		}
		
		switch(state){
			case stopped:
				break;

			case running:		

				if(TCC0.INTFLAGS & 0x01){
					for(i = 0; i < 3; i ++){
						rollcash[i] = 0;
						accelcash[i] = 0;
					}
					TCC0.INTFLAGS = 0x01;
					do{
						while(imu.status != TWIM_STATUS_READY);
						TWI_MasterWriteRead(&imu, ROLL, &rollstartbyte, 1, 10);
						while(imu.result == TWIM_RESULT_UNKNOWN);
					}while(!(imu.readData[0] & 0x01));
					for(i = 0; i < 5; i += 2){
						rollcash[i/2] += ((char)(imu.readData[i + 3]));
					}

					PORTF.OUT = 1;
					do{
						while(imu.status != TWIM_STATUS_READY);
						TWI_MasterWriteRead(&imu, ACCEL, &accelstartbyte, 1, 10);
						while(imu.result == TWIM_RESULT_UNKNOWN);
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

					sprintf(xbeebuffer, "%d %d\n\r", rollcash[0], accelcash[0]);
					sendstring(&xbee, xbeebuffer);
				}
				break;

		}
	}
	return 0;
}
ISR(USARTE0_RXC_vect){
	USART_RXComplete(&xbee);
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&imu);
}
