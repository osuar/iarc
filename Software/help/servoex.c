/**

@author Daniel Sidlauskas Miller

Example xmega pwm (pulse width modulation code) for servo control

*/

/**Include avr compliler header (contains register definitions for xmega chip) and standard lib (basic functions like logic)*/

#include <stdlib.h>
#include "avr_compiler.h"

/**Constants to easily getclear overflow flags, trigger when timer counter reaches period*/
#define TC_GetOverflowFlag( _tc ) ( (_tc)->INTFLAGS & TC0_OVFIF_bm )
#define TC_ClearOverflowFlag( _tc ) ( (_tc)->INTFLAGS = TC0_OVFIF_bm )

/**timer counter delay function prototype*/
void tcdelay( int cycles );

int main( void )
{
	/**Set pin1 in portc to ouput, case sensitive*/
	PORTC.DIR = 0x01;

	/**Configure Timer Counter C0 for 50hz pwm operation, TCC0 has 4 compare channels (pin1 - pin4) labled CCA, CCB...*/
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc; //2MHz clock speed
	TCC0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm; //Set single slope wave generation mode, enable CCA
	TCC0.PER = 40000; //2MHz / 40kHz = 50Hz

	/** enter infinite loop alternating servo position*/
	while(1)
	{
		/**Set Compare Channel A (CCA) value to establish pwm signal*/
		TCC0.CCA = 2500;
		tcdelay( 10 );

		TCC0.CCA = 3000;
		tcdelay( 10 );

		TCC0.CCA = 3500;
		tcdelay( 10 );
	}

	return 0;
}

void tcdelay( int cycles)
{
	int i;
	for(i = 0; i < cycles; i++)
	{
		while(0 == TC_GetOverflowFlag( &TCC0 ));
		TC_ClearOverflowFlag( &TCC0 );
	}
	return;
}
		
