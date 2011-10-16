/**

@author Daniel Sidlauskas Miller

Flight control program, start Mar23 2011
*/

#include <util/delay.h>
#include "avr_compiler.h"
#include <stdlib.h>
#include "twi_master_driver.h"
#include "usart_driver.h"
#include "support.h"

TWI_Master_t imu;

USART_data_t xbee;

volatile int readdata = 0;

int main ( void ) {
	
	/**Support Vars*/
	int i;
	int rollflag = 0;
	int accelflag = 1;
	int bytetobuffer;

	/**IMU setup buffer*/
	uint8_t accelsetupbuffer[2] = {0x2D, 0x08};
	uint8_t accelstartbyte = 0x30;
	uint8_t rollsetupbuffer[2] = {0x16, 0x18};
	uint8_t rollstartbyte = 0x1A;

	/**Setup directions for serial interfaces*/
	PORTC.DIR = 0b00101100;
	PORTC.OUT |= 0b00001000;
	PORTD.DIR |= 0b00100000;
	PORTF.DIR |= 0x01;
	
	/**Enable pullup resistors for imu*/
	PORTCFG.MPCMASK = 0x03;
	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc;

	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	/**Setup IMU*/
	TWI_MasterInit(&imu, &TWIC, TWI_MASTER_INTLVL_LO_gc, TWI_BAUDSETTING);

	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ACCEL, accelsetupbuffer, 2 , 0);

	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer, 2, 0);
	while(imu.status != TWIM_STATUS_READY);	

	/**Setup Xbee*/
	USART_InterruptDriver_Initialize(&xbee, &USARTD0, USART_DREINTLVL_HI_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTD0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);

	/**control loop*/
	while(1){
		/**If imu ready, then read, alternate accel and roll*/
		if((imu.status == TWIM_STATUS_READY) && !readdata){
			if(rollflag){
				rollflag = 0;
				accelflag = 1;
				TWI_MasterWriteRead(&imu, ROLL, &rollstartbyte, 1, 9);
			}
			else if(accelflag){
				accelflag = 0;
				rollflag = 1;
				TWI_MasterWriteRead(&imu, ACCEL, &accelstartbyte, 1, 8);
			}
		}
		
		if(readdata){
			readdata = 0;
			/**Roll read if accelflag set*/
			if(accelflag){
				for(i = 0; i < 9; i ++){
					bytetobuffer = 0;
					while(!bytetobuffer){
						bytetobuffer = USART_TXBuffer_PutByte(&xbee, imu.readData[i]);
					}
				}
			}
			else if(rollflag){
				PORTF.OUT ^= 0x01;
				for(i = 0; i < 8; i ++){
					bytetobuffer = 0;
					while(!bytetobuffer){
						bytetobuffer = USART_TXBuffer_PutByte(&xbee, imu.readData[i]);
					}
				}
			}
		}
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
		for(i = 0; i < 10000; i ++);
	}
	return 0;
}

ISR(USARTD0_RXC_vect){
	USART_RXComplete(&xbee);
}

ISR(USARTD0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}

ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&imu);
	readdata = 1;
}	 
