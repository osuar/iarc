#include "twi_master_driver.h"
#include "dac_driver.h"
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>


#define CPU_SPEED 2000000
#define BAUDRATE 100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

#define Roll (0xA6 >> 1)

TWI_Master_t twiMaster;

volatile int readyflag = 0;
volatile int dataready = 0;

int main( void )
{
	uint8_t RollSetupBuffer[2];
	RollSetupBuffer[0] = 0x2D;
	RollSetupBuffer[1] = 0x08;

	uint8_t RollRead[2] = {0x32, 0x30};

	uint8_t RollReadBuffer[2];
	
	int convertcount;
	
	PORTA.DIR = 0xff;
	PORTD.DIR = 0xff;
	PORTB.DIR = 0xff;
	PORTF.DIR = 0x01;
	
	//IMU Power
	PORTC.DIR = (1 << 2) | (1 << 3);
	PORTC.OUT = (1 << 3);
	
	//Pullup Resistors
	PORTCFG.MPCMASK = 0x03;
	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc;

	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();

	TWI_MasterInit(&twiMaster, &TWIC, TWI_MASTER_INTLVL_LO_gc, TWI_BAUDSETTING);
	while(twiMaster.status != TWIM_STATUS_READY){}
	TWI_MasterWriteRead(&twiMaster, Roll, RollSetupBuffer, 2, 0);
	while(twiMaster.status != TWIM_STATUS_READY){}

	while(1)
	{
		TWI_MasterWriteRead(&twiMaster, Roll, &RollRead[1], 1, 8);

		_delay_ms(100);
		PORTF.OUT ^= 0x01;

		while(twiMaster.status != TWIM_STATUS_READY){}
		if((twiMaster.readData[0] & 0x80) == 0x80)
		{
		PORTA.OUT = twiMaster.readData[2];
		}
	}

	return 0;
}

/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}
