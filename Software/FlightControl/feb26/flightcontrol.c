/**
@author Daniel Sidlauskas Miller with advisement from Gary Miller

Flight control program for Oregon State University Robitcs Club Autonomous Aerial Team

Sorbothane
*/

#include <stdlib.h>
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "aux.h"

//
TWI_Master_t imu;

volatile int servol = SERVOINI;
volatile int motorl = MOTORINI;
volatile int servor = SERVOINI;
volatile int motorr = MOTORINI;

volatile uint8_t checkvalue;

volatile int readycheckflag = 0;

volatile int dataflag = 0;


int main( void )
{
	//Accelerometer buffer vars
	uint8_t accelsetupbuffer[2] = {0x2D, 0x08};
	uint8_t accelstartbyte[2] = {0x30, 0x32};
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
	TCC1.PER = 2857;

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

	//delay for motors to initialize
	for(datatransfer = 0; datatransfer <= 100; datatransfer++)
	{
		while(TC_GetOverflowFlag( &TCD0 ) == 0){}
		TC_ClearOverflowFlag( &TCD0 );
	}

	//infinite loop
	while(1)
	{
		if((imu.status == TWIM_STATUS_READY) && (2 == readycheckflag))
		{
			TWI_MasterWriteRead(&imu, ACCEL, &accelstartbyte[1], 1, 6);
		}
		//if TCD0 overflow flag set, ask for more IMU data (update once per motor pwm cycle)
		if((TC_GetOverflowFlag( &TCC1 ) != 0) && (readycheckflag == 0))
		{
			while(imu.status != TWIM_STATUS_READY);
			TC_ClearOverflowFlag( &TCC1 );
			TWI_MasterWriteRead(&imu, ACCEL, &accelstartbyte[0], 1, 1);
			readycheckflag = 1;
		}

		//if new accel data received, process new data
		if(1 == dataflag)
		{
			dataflag = 0;			
			imuread ++;
			readycheckflag = 0;

			//Process 2 bytes for X, Y, Z
			for(datatransfer = 0; datatransfer <= 4; datatransfer += 2)
			{
				//if twos compliment bit set, flip bits and subtract
				if((imu.readData[datatransfer + 1] & 0x80) == 0x80)
				{
					accelreadbuffer[datatransfer / 2] -= ((~imu.readData[datatransfer] + 1) & ~(0x00));
					accelreadbuffer[datatransfer / 2] -= (256 * (~imu.readData[datatransfer + 1] + 1));
				}
				else
				{
					accelreadbuffer[datatransfer / 2] += (imu.readData[datatransfer] & ~(0x00));
					accelreadbuffer[datatransfer / 2] += 256 * imu.readData[datatransfer + 1];
				}			
			}
			//If read four times (should read 4 times per motor pwm cycle), average data (divide by 4)
			if(7 <= imuread)
			{
				for(datatransfer = 0; datatransfer <= 2; datatransfer ++)
				{
					accelreadbuffer[datatransfer] = (accelreadbuffer[datatransfer] / 7);
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
			accelreadbuffer[0] -= AXNORM;
			accelreadbuffer[2] -= AZNORM;
			servol = ServoLValueFunk(accelreadbuffer[0], accelreadbuffer[1], accelreadbuffer[2]);
			servor = ServoRValueFunk(accelreadbuffer[0], accelreadbuffer[1], accelreadbuffer[2]);
			motorl = MotorLValueFunk(accelreadbuffer[0], accelreadbuffer[1], accelreadbuffer[2]);
			motorr = MotorRValueFunk(accelreadbuffer[0], accelreadbuffer[1], accelreadbuffer[2]);
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
	//TCD0.CCB = motorl;
}

ISR(TCD0_CCC_vect)
{
	TCD0.CCC = servor;
}

ISR(TCD0_CCD_vect)
{
	//TCD0.CCD = motorr;
}

//IMU data received interrupt
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&imu);
	if(imu.readData[0] != checkvalue)
	{
		PORTD.OUT ^= 0x80;
	}
	checkvalue = imu.readData[0];
	if(1 == readycheckflag)
	{		
		if((imu.readData[0] & 0x80) == 0x80)
		{
			readycheckflag = 2;
		}
		else
		{
			readycheckflag = 0;
		}
	}
	else if(2 == readycheckflag)
	{
	dataflag = 1;
	}
}	
