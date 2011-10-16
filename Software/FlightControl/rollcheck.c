#include <stdio.h>
#include "avr_compiler.h"
#include "usart_driver.h"
#include "twi_master_driver.h"
#include "support.h"

volatile char readdata = 0;
volatile char setup = 1;
volatile char imuflag = 0;

TWI_Master_t imu;

USART_data_t xbee;

int main(void){

	int translen = 0;
	char xbeebuffer[100];
	int i;
	int bytetobuffer;
	char rollread = 0;
	char accelread = 0;
	int dataready = 0;
	char receive;
	char count = 0;

	uint8_t accelsetupbuffer[3] = {0x2C, 0b00001100, 0x08};
	uint8_t accelstartbyte = 0x30;
	uint8_t rollsetupbuffer1[4] = {0x15, 0x04, 0x19, 0x11};
	uint8_t rollsetupbuffer2[] = {0x3E, 0b00000001};
	uint8_t rollstartbyte = 0x1A;
	
	char rollcash[3] = {0,0,0};
	int accelcash[3] = {0,0,0};

	short int motorr = 0;
	short int motorl = 0;
	short int servor = 0;
	short int servol = 0;

	enum states {running, stopped} state = stopped;

	/**Setup directions for serial interfaces*/
	PORTC.DIR = 0b00001000;
	PORTC.OUT = 0b00001000;
	PORTE.DIR = 0b00001000;
	PORTF.DIR = 0x03;
	PORTD.DIR = 0x0F;

	//Pulse width modulation setup for servos, port D
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 40000;

	/**Enable pullup resistors for imu*/
	PORTCFG.MPCMASK = 0x03;
	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc;

	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	/**Setup IMU*/
	TWI_MasterInit(&imu, &TWIC, TWI_MASTER_INTLVL_HI_gc, TWI_BAUDSETTING);

	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ACCEL, accelsetupbuffer, 3, 0);
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

	setup = 0;
	readdata = 0;
	while(1){


		if(USART_RXBufferData_Available(&xbee)){
			receive = USART_RXBuffer_GetByte(&xbee);
			if(receive == 's'){
				state = running;
			}
			else if(receive == 'n'){
				state = stopped;
			}
		}

		switch(state){
		case stopped:
		PORTF.OUT = 3;
		TCD0.CCA = 2000;
		TCD0.CCC = 2000;
		TCD0.CCB = 3000;
		TCD0.CCD = 3000;
		for(i = 0; i < 3; i ++){
			accelcash[i] = 0;
			rollcash[i] = 0;
		}
		break;

		case running:
		PORTF.OUT = 0;
		//Roll reading
		while(imu.status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(&imu, ROLL, &rollstartbyte, 1, 10);
		while(!readdata);
		readdata = 0;
		PORTF.OUT |= 0x01;
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



		//Accel reading
		while(imu.status != TWIM_STATUS_READY);
		PORTF.OUT ^= 1;
		TWI_MasterWriteRead(&imu, ACCEL, &accelstartbyte, 1, 10);
		while(!readdata);
		readdata = 0;

		for(i = 0; i < 5; i += 2){
				rollcash[i/2] += ((char)(imu.readData[i + 3]));
		}
		PORTF.OUT |= 0x02;

		count ++;
		if(count > 4){
			for(i = 0; i < 3; i ++){
				accelcash[i] /= 5;
				rollcash[i] /= 2;
			}


		//motor updates
			rollcash[0] -= RXN;
			rollcash[1] -= RYN;
			rollcash[2] -= RZN;
			accelcash[0] -= AXN;
			accelcash[1] -= AYN;
			accelcash[2] -= AZN;


			ValueFunk(accelcash[0],accelcash[1],accelcash[2],rollcash[0],rollcash[1],rollcash[2],&servol,&servor,&motorl,&motorr);
			while(TCD0.CNT < 4000);

			TCD0.CCA = motorr;
			TCD0.CCB = servor;
			TCD0.CCC = motorl;
			TCD0.CCD = servol;

			sprintf(xbeebuffer, " X%4d Y%4d Z%4d x%4d y%4d z%4d R%4d r%4d L%4d l%4d\n\r", rollcash[0], rollcash[1], rollcash[2], accelcash[0], accelcash[1], accelcash[2], motorr, servor, motorl, servol);
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
			for(i = 0; i < 3; i ++){
				accelcash[i] = 0;
			}
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
	if(!setup){
		readdata = 1;
	}
}	 

ISR(BADISR_vect){
	while(1){
		PORTF.OUT ^= 0x03;
		_delay_ms(100);
	}
}

