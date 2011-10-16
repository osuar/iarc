#include <stdlib.h>
#include "avr_compiler.h"
#include "usart_driver.h"
#include "support.h"
#include <stdio.h>

USART_data_t xbee;

int main(void){
	char input;

/*
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
*/

	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

/*
	Setup IMU
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
*/

	/**Setup Xbee*/
	PORTE.DIR = 0b00001000;
	PORTF.DIR = 1;
	
	USART_InterruptDriver_Initialize(&xbee, &USARTE0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);
	

	while(1){
		if(USART_RXBufferData_Available(&xbee)){
			PORTF.OUT ^= 0x01;
			input = USART_RXBuffer_GetByte(&xbee);
			sendchar(&xbee, input);
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
