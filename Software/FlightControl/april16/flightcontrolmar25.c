/**
@author Daniel Sidlauskas Miller with advisement from Gary Miller

Flight control program for Oregon State University Robitcs Club Autonomous Aerial Team

Sorbothane
*/

#include <stdlib.h>
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "support.h"
#include <stdio.h>
#include "usart_driver.h"

//
TWI_Master_t imu;

volatile int servol = SERVOINI;
volatile int motorl = MOTORINI;
volatile int servor = SERVOINI;
volatile int motorr = MOTORINI;

volatile char imuflag = 1;
volatile char dataflag = 0;

USART_data_t xbee;

int main( void )
{
	//Accelerometer buffer vars
	uint8_t accelsetupbuffer[2] = {0x2D, 0x08};
	uint8_t accelstartbyte = 0x30;
	signed short int accelreadbuffer[3];
	uint8_t rollsetupbuffer1[2] = {0x16, 0x18};
	uint8_t rollsetupbuffer2[2] = {0x17, 0x01};
	uint8_t rollstartbyte = 0x1A;
	signed short int rollreadbuffer[3];
	int acceltotal[] = {0,0,0};
	int rolltotal[] = {0,0,0};

	//flags and transfers
	int newdata = 0;
	int datatransfer = 0;
	int imuread = 0;
	int datacounter = 0;
	int i;
	int rollcash[3] = {0, 0, 0};
	int accelcash[3] = {0, 0, 0};
	int motorcash[4] = {0, 0, 0, 0};
	int translen = 0;
	int transcount = 1;
	char xbeebuffer[100];
	int bytetobuffer;
	char receive;

	//Set PORTD and PORTF to ouput for signaling
	PORTD.DIR = 0xff;
	PORTE.DIR = 0x08;
	PORTF.DIR = 0x01;

	//Enable interrupts
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();	

	//Pulse width modulation setup for servos, port D
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 40000;

	//Timer TC enable to update imu data 4 times per motor updata
	TCC1.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC1.CTRLB = TC_WGMODE_SS_gc | TC1_CCAEN_bm;
	TCC1.PER = 2000;


	//Set interrupt to update each motor value
	TCD0.INTCTRLB = TC_CCAINTLVL_HI_gc | TC_CCBINTLVL_HI_gc | TC_CCCINTLVL_HI_gc | TC_CCDINTLVL_HI_gc;

	//Enable pullup resistors for imu
	PORTCFG.MPCMASK = 0x03;
	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc;

	//Start Two Wire interface with IMU on PORTC (PIN1 = SDA, PIN2 = SCL, PIN4 = 3.3V, PIN3 = GND)
	PORTC.DIR = 0x0C;
	PORTC.OUT = 0x08;	
	TWI_MasterInit(&imu, &TWIC, TWI_MASTER_INTLVL_LO_gc, TWI_BAUDSETTING);	
	
	//Setup Accelerometer
	while(imu.status != TWIM_STATUS_READY){}
	TWI_MasterWriteRead(&imu, ACCEL, accelsetupbuffer, 2, 0);
	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer1, 2, 0);
	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer2, 2, 0);

	/**Setup Xbee*/

	USART_InterruptDriver_Initialize(&xbee, &USARTE0, USART_DREINTLVL_HI_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 12 , 0); 
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);

	//delay for motors to initialize
	for(datatransfer = 0; datatransfer <= 100; datatransfer++)
	{
		while(TC_GetOverflowFlag( &TCD0 ) == 0){}
		TC_ClearOverflowFlag( &TCD0 );
		PORTF.OUT ^= 0x01;
	}

	PORTF.OUT = 0;
	//infinite loop
	while(1)
	{
		_delay_ms(10);

		if(USART_RXBufferData_Available(&xbee)){
			receive = USART_RXBuffer_GetByte(&xbee);
			if('q' == receive){
				TCD0.CCB = 2000;
				TCD0.CCD = 2000;
				motorl = 2000;
				motorr = 2000;
				return 0;
			} 
		}
/*
		if(dataflag){
			translen = sprintf(xbeebuffer, "  %d\n\r  ", imuread);
			for(i = 0; i < translen; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
		}
*/

		if(datacounter == 4){ 
			PORTF.OUT = 1;
			for(i = 0; i < 3; i ++){
				accelcash[i] /= 4;
				motorcash[i] /= 4;
			}
			motorcash[3] /= 4;
			//captial means left, lower case means right, lower case means Accel, upper means roll
			translen = sprintf(xbeebuffer, "  X%dY%dZ%dx%dy%dz%dS%ds%dM%dm%d\n\r  ",rollcash[0], rollcash[1], rollcash[2], accelcash[0], accelcash[1], accelcash[2], motorcash[0], motorcash[1], motorcash[2], motorcash[3]);
			transcount = 0;
			for(i = 0; i < 3; i ++){
				accelcash[i] = 0;
				rollcash[i] = 0;
				motorcash[i] = 0;
			}
			motorcash[3] = 0;
			datacounter = 0;
		}
		if(transcount < translen){
			bytetobuffer = 0;
			while(!bytetobuffer){
			bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[transcount]);
			}
				transcount ++;
		}

		//if TCD0 overflow flag set, ask for more IMU data (update once per motor pwm cycle)
		if((1 == TC_GetOverflowFlag(&TCC1)) && (imu.status == TWIM_STATUS_READY) && !dataflag){
			TC_ClearOverflowFlag(&TCC1);
			if(!imuflag){
				TWI_MasterWriteRead(&imu, ROLL, &rollstartbyte, 1, 9);
			}
			else if(imuflag){
				TWI_MasterWriteRead(&imu, ACCEL, &accelstartbyte, 1, 8);
			}
		}

		//if new accel data received, process new data
		if(1 == dataflag)
		{
			dataflag = 0;			
			imuread ++;

			if(!imuflag){
				//Process 2 bytes for X, Y, Z
				for(datatransfer = 2; datatransfer <= 6; datatransfer += 2)
				{
					//if twos compliment bit set, flip bits and subtract
					if((imu.readData[datatransfer + 1] & 0x80) == 0x80)
					{
						accelreadbuffer[-1 + (datatransfer / 2)] -= ((~imu.readData[datatransfer]) & ~(0x00));
						accelreadbuffer[-1 + (datatransfer / 2)] -= (256 * (~imu.readData[datatransfer + 1] + 1));
					}
					else
					{
						accelreadbuffer[-1 + (datatransfer / 2)] += (imu.readData[datatransfer] & ~(0x00));
						accelreadbuffer[-1 + (datatransfer / 2)] += 256 * imu.readData[datatransfer + 1];
					}			
				}
			}
			if(imuflag){
				for(datatransfer = 0; datatransfer <= 4; datatransfer += 2){
					if(imu.readData[datatransfer + 3] & 0x80){
					//	rollreadbuffer[datatransfer / 2] -= (~imu.readData[datatransfer + 4] + 1);
						rollreadbuffer[datatransfer / 2] -= (~imu.readData[datatransfer + 3] + 1);
					}
					else
					{
					//	rollreadbuffer[datatransfer / 2] += imu.readData[datatransfer + 4];
						rollreadbuffer[datatransfer / 2] += imu.readData[datatransfer + 3];
					}
				}
			}
			//If read four times (should read 4 times per motor pwm cycle), average data (divide by 4)
			if(40 <= imuread)
			{
				for(datatransfer = 0; datatransfer <= 2; datatransfer ++)
				{
					accelreadbuffer[datatransfer] = (accelreadbuffer[datatransfer] / 20);
					rollreadbuffer[datatransfer] /= 20;
				}
				newdata = 1;
				imuread = 0;
			}
			
		}

		//if new data, update motor compare values
		if(1 == newdata)
		{
			newdata = 0;
			accelreadbuffer[0] -= AXNORM;
			accelreadbuffer[2] -= AZNORM;

			ValueFunk(accelreadbuffer[0], accelreadbuffer[1], accelreadbuffer[2], rollreadbuffer[0], rollreadbuffer[1], rollreadbuffer[2], &servol, &servor, &motorl, &motorr);

			//X axis modification due to force of gravity
			for(datatransfer = 0; datatransfer < 3; datatransfer ++){
				accelcash[datatransfer] += accelreadbuffer[datatransfer];
				rollcash[datatransfer] += rollreadbuffer[datatransfer];
				acceltotal[datatransfer] += accelreadbuffer[datatransfer];
				rolltotal[datatransfer] += rollreadbuffer[datatransfer];
				accelreadbuffer[datatransfer] = 0;
				rollreadbuffer[datatransfer] = 0;
			}
			
			motorcash[0] += servol;
			motorcash[1] += servor;
			motorcash[2] += motorl;
			motorcash[3] += motorr;

			datacounter ++;

		}
	}
	return 0;
}

//Motor update interrupts
ISR(TCD0_CCA_vect)
{
	TCD0.CCA = servol;
}

ISR(TCD0_CCB_vect)
{
	TCD0.CCB = motorl;
}

ISR(TCD0_CCC_vect)
{
	TCD0.CCC = servor;
}

ISR(TCD0_CCD_vect)
{
	TCD0.CCD = motorr;
}

//IMU data received interrupt
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&imu);
	if(((imu.readData[0] & 0x80) && imuflag) || ((imu.readData[0] & 0x01) && !imuflag))
	{
		imuflag ^= 1;
		PORTD.OUT ^= 0x80;
		dataflag = 1;
	}
}
ISR(USARTE0_RXC_vect){
	USART_RXComplete(&xbee);
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
