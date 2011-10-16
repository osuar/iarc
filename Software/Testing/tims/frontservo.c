/**

@author Kyle Dillon

This is the servo code for the front IR. It spans about 110 degrees. Takes 10 samples (stops 10 times). And takes about 1 second per span. This is not the final code and can be adjusted if need be in the future. 

PORTD.DIR = 0b00000000;
PORTD.OUT = 0b00000000;
_delay_ms(x);

*/


/**Include avr compliler header (contains register definitions for xmega chip) and standard lib (basic functions like logic)*/

#include <stdlib.h>
#include "avr_compiler.h"

/**Constants to easily getclear overflow flags, trigger when timer counter reaches period*/
#define TC_GetOverflowFlag( _tc ) ( (_tc)->INTFLAGS & TC0_OVFIF_bm )
#define DELAY 5

/**timer counter delay function prototype*/
void tcdelay( int cycles );

int main( void )
{
	/**Set pin0 in portc to ouput, case sensitive*/
	PORTC.DIR = 0x01;
	int compare = 2000;	//starting compare value. 
	int m, i, j, k, l;

	/**Configure Timer Counter C0 for 50hz pwm operation, TCC0 has 4 compare channels (pin1 - pin4) labled CCA, CCB...*/
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc; //2MHz clock speed
	TCC0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm; //Set single slope wave generation mode, enable CCA
	TCC0.PER = 40000; //2MHz / 40kHz = 50Hz
	int delay = 222222222222222222222200;	
/** enter infinite loop alternating servo position*/
	while(1){
		for(m=0; m<100; m++){
			PORTD.DIR = 0b11110000;
			PORTD.OUT = 0b10000000;
			tcdelay(delay);
			PORTD.OUT ^= 0b10100000;
			tcdelay(delay);
			PORTD.OUT ^= 0b01100000;
			tcdelay(delay);
			PORTD.OUT ^= 0b01010000;
			tcdelay(delay);
			PORTD.OUT ^= 0b10010000;
		/*	for(i=0; i<4; i++){
				for(j=0; j<3; j++){
					for(k=0; k<2; k++){
						for(l=0; l<1; l++){
							PORTD.OUT ^= 0b00010000;
							tcdelay(delay);
						}
						PORTD.OUT ^= 0b01000000;
						tcdelay(delay);
					}
					PORTD.OUT ^= 0b00100000;
					tcdelay(delay);
				}
				PORTD.OUT ^= 0b10000000;
				tcdelay(delay);
			}
	*/	}
	}
	
/*
	while(1)
	{
		Set Compare Channel A (CCA) value to establish pwm signal
		
		i = 0;
		while(i<11)	// stops 10 times.
		{
		TCC0.CCA = (compare + (i*320));		//increases the compare value by increments of 320.
		tcdelay( DELAY );
		i++;
		}

		
	}
*/

	return 0;
}

void tcdelay( int cycles)
{
	int i;
	for(i = 0; i < cycles; i++)
	{
		while(0 == TC_GetOverflowFlag( &TCC0 ));
		TC_GetOverflowFlag( &TCC0 );
	}
	return;
}
		
