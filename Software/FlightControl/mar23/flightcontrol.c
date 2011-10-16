/**
@author Daniel Sidlauskas Miller with advisement from Gary Miller

Flight control program for Oregon State University Robitcs Club Autonomous Aerial Team

Sorbothane
*/

#include <stdlib.h>
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "aux.h"
#include <stdio.h>

//
TWI_Master_t imu;

volatile int servol = SERVOINI;
volatile int motorl = MOTORINI;
volatile int servor = SERVOINI;
volatile int motorr = MOTORINI;

volatile int dataflag = 0;


int main( void )
{
	//Accelerometer buffer vars
	uint8_t accelsetupbuffer[2] = {0x2D, 0x08};
	uint8_t accelstartbyte = 0x30;
	signed int accelreadbuffer[3];

	//flags and transfers
	int newdata = 0;
	int datatransfer = 0;
	int imuread = 0;

	//Set PORTD and PORTF to ouput for signaling
	PORTD.DIR = 0xff;
	PORTF.DIR = 0x01;

	//Enable interrupts
	PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();	

	//Pulse width modulation setup for servos, port D
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 40000;

	//Timer TC enable to update imu data 4 times per motor updata
	TCC1.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC1.CTRLB = TC_WGMODE_SS_gc | TC1_CCAEN_bm;
	TCC1.PER = 2000;

	PORTA.DIR = 0xFF;

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

	/**Setup Xbee*/
	USART_InterruptDriver_Initialize(&xbee, &USARTD0, USART_DREINTLVL_HI_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTD0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);

	//delay for motors to initialize
	for(datatransfer = 0; datatransfer <= 100; datatransfer++)
	{
		while(TC_GetOverflowFlag( &TCD0 ) == 0){}
		TC_ClearOverflowFlag( &TCD0 );
		PORTF.OUT ^= 0x01;
	}

	//infinite loop
	while(1)
	{
		//if TCD0 overflow flag set, ask for more IMU data (update once per motor pwm cycle)
		if(TC_GetOverflowFlag( &TCC1 ) != 0)
		{
			//if status not  ready, call for help
			while(imu.status != TWIM_STATUS_READY)
			{
				PORTD.OUT ^= 0x40;
			}

			TC_ClearOverflowFlag( &TCC1 );
			TWI_MasterWriteRead(&imu, ACCEL, &accelstartbyte, 1, 8);
		}

		//if new accel data received, process new data
		if(1 == dataflag)
		{
			PORTA.OUT = imu.readData[2];
			dataflag = 0;			
			imuread ++;

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
			//If read four times (should read 4 times per motor pwm cycle), average data (divide by 4)
			if(20 <= imuread)
			{
				for(datatransfer = 0; datatransfer <= 2; datatransfer ++)
				{
					accelreadbuffer[datatransfer] = (accelreadbuffer[datatransfer] / 20);
				}
				newdata = 1;
				imuread = 0;
			}
		}

		//if new data, update motor compare values
		if(1 == newdata)
		{
			newdata = 0;
			//X axis modification due to force of gravity
			radlen1 = sprintf(radiobuff1, "x:%d,y:%d,z:%d,", accelreadbuffer[0], accellreadbuffer[1], accelreadbuffer[2]);
			accelreadbuffer[0] -= AXNORM;
			accelreadbuffer[2] -= AZNORM;
			servol = ServoLValueFunk(accelreadbuffer[0], accelreadbuffer[1], accelreadbuffer[2]);
			servor = ServoRValueFunk(accelreadbuffer[0], accelreadbuffer[1], accelreadbuffer[2]);
			motorl = MotorLValueFunk(accelreadbuffer[0], accelreadbuffer[1], accelreadbuffer[2]);
			motorr = MotorRValueFunk(accelreadbuffer[0], accelreadbuffer[1], accelreadbuffer[2]);
			radlen2 = sprintf(radiobuff2, "sl:%d,sr:%d,ml:%d,mr:%d\n", serol, servor, motorl, motorr);
			accelreadbuffer[0] = 0;
			accelreadbuffer[1] = 0;
			accelreadbuffer[2] = 0;
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
	if((imu.readData[0] & 0x80) == 0x80)
	{
		PORTD.OUT ^= 0x80;
		dataflag = 1;
	}
}
