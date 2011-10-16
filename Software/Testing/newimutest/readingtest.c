#include <stdio.h>
#include "avr_compiler.h"
#include "usart_driver.h"
#include "twi_master_driver.h"
#include "support.h"

TWI_Master_t imu;

USART_data_t xbee;

volatile int readdata = 0;

int main(void){
	int i;
	int bytetobuffer;
	char xbeebuffer[100];
	char receive;

	uint8_t rollsetupbuffer1[4] = {0x15, 0x04, 0x19, 0x11};
	uint8_t rollsetupbuffer2[] = {0x3E, 0b00000001};
	uint8_t rollstartbyte = 0x1A;
	
	enum states {runningwrite, runningread, stopped} state = stopped;

	/**Setup directions for serial interfaces*/
	PORTC.DIR = 0b00001100;
	PORTC.OUT |= 0b00001000;
	PORTE.DIR |= 0b00001000;
	PORTF.DIR |= 0x03;
	PORTD.DIR = 0x0F;

	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	/**Setup IMU*/
	TWI_MasterInit(&imu, &TWIC, TWI_MASTER_INTLVL_HI_gc, TWI_BAUDSETTING);


	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer1, 4, 0);
	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer2, 2, 0);
	while(imu.status != TWIM_STATUS_READY);

	/**Setup Xbee*/
	USART_InterruptDriver_Initialize(&xbee, &USARTE0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);

	while(1){

	_delay_ms(100);
	PORTF.OUT ^= 0x01;

	if(USART_RXBufferData_Available(&xbee)){
		receive = USART_RXBuffer_GetByte(&xbee);
		if(receive == 'r'){
			state = runningread;
			sprintf(xbeebuffer, "read\n\r");
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
		}
		else if(receive == 'w'){
			state = runningwrite;
			sprintf(xbeebuffer, "write\n\r");
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
		}
		else if(receive == 's'){
			state = stopped; 
			sprintf(xbeebuffer, "stop\n\r");
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
		}

	}

	switch(state){

	case stopped:

	break;	

	case runningread:
		while(imu.status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(&imu, ROLL, &rollstartbyte, 1, 10);
		while(!readdata);
		readdata = 0;
	break;

	case runningwrite:
		while(imu.status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(&imu, ROLL, &rollstartbyte, 1, 0 );
		while(!readdata);
		readdata = 0;
	break;
	}


	}

}

ISR(USARTE0_RXC_vect){
	USART_RXComplete(&xbee);
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}

ISR(TWIC_TWIM_vect){
	if(TWI_MasterInterruptHandler(&imu)){
	readdata = 1;
	}
	else{
	}
}	 

