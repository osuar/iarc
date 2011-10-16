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
#include <stdio.h>

TWI_Master_t imu;

USART_data_t xbee;

volatile int readdata = 0;
volatile int imuflag = 1;
volatile char setup = 1;

int main ( void ) {
	
	/**Support Vars*/
	short int i;
	char imuread = 0;
	char bytetobuffer;
	short int transcount = 1;
	char translen = 0;

	/**xbee vars*/
	char xbeebuffer[100];

	/**IMU setup buffer*/
	uint8_t accelsetupbuffer[3] = {0x2C, 0b00001011, 0x08};
	uint8_t accelstartbyte = 0x30;
	uint8_t rollsetupbuffer1[4] = {0x15, 0x01, 0x19, 0x11};
	uint8_t rollsetupbuffer2[] = {0x3E, 0b00000001};
	uint8_t rollstartbyte = 0x1A;

	/**imu read*/
	short signed int accelcash[] = {0,0,0};
	short signed int rollcash[] = {0,0,0};

	/**Setup directions for serial interfaces*/
	PORTC.DIR = 0b00101100;
	PORTC.OUT |= 0b00001000;
	PORTE.DIR |= 0b00001000;
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
	TWI_MasterWriteRead(&imu, ACCEL, accelsetupbuffer, 3 , 0);
	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer1, 4, 0);
	while(imu.status != TWIM_STATUS_READY);	
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer2, 2, 0);
	while(imu.status != TWIM_STATUS_READY);

	/**Setup Xbee*/
	USART_InterruptDriver_Initialize(&xbee, &USARTE0, USART_DREINTLVL_HI_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);
	
	setup = 0;
	/**control loop*/
	while(1){
		/**If imu ready, then read, alternate accel and roll*/
		if(((imu.status == TWIM_STATUS_READY) && !readdata)){
			if(!imuflag){
				PORTF.OUT = 1;
				TWI_MasterWriteRead(&imu, ROLL, &rollstartbyte, 1, 9);
			}
			else if(imuflag){
				_delay_ms(91);
				TWI_MasterWriteRead(&imu, ACCEL, &accelstartbyte, 1, 8);
			}
		}
		
		if(readdata){
			readdata = 0;
			imuread ++;
			/**Roll read if accelflag set*/
			if(imuflag){
				for(i = 0; i <= 4; i += 2){
					if((imu.readData[i + 3] & 0x80) == 0x80){
						rollcash[i/2] -= ((~imu.readData[i + 3]) + 1);
						//rollcash[i/2] -= ~imu.readData[i + 4] + 1;
					}
					else{
						rollcash[i/2] += (imu.readData[i + 3]);
						//rollcash[i/2] += imu.readData[i + 4];
					}
				}
			}
			else if(!imuflag){
				for(i = 0; i <= 4; i += 2){
					if((imu.readData[i + 3] & 0x80) == 0x80){
						accelcash[i/2] -= 256 * (~imu.readData[i + 3] + 1);
						accelcash[i/2] -= ~imu.readData[i + 2] + 1;
					}
					else{
						accelcash[i/2] += 256 * imu.readData[i + 3];
						accelcash[i/2] += imu.readData[i + 2];
					}
				}
			}
		}
		if(imuread >= 20){
			imuread = 0;
			for(i = 0; i < 3; i ++){
				accelcash[i] /= 10;
				rollcash[i] /= 10;
			}
			if(translen <= transcount){
				translen = sprintf(xbeebuffer, "  X%dY%dZ%dx%dy%dz%d\n\r  ", rollcash[0], rollcash[1], rollcash[2], accelcash[0], accelcash[1], accelcash[2]);
			transcount = 0;
			}
			for(i = 0; i < 3; i ++){
				accelcash[i] = 0;
				rollcash[i] = 0;
			}
		}
		if(translen > transcount){
			bytetobuffer = 0;
			while(!bytetobuffer){
				bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[transcount]);
			}
			transcount ++;
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
	if(!setup){
			if((imuflag && ((imu.readData[0] & 0x80) == 0x80)) || (!imuflag && ((imu.readData[0] & 0x01) == 1))) {
			PORTF.OUT = 1;
			imuflag ^= 1;
			readdata = 1;
		}
	}
}	 
