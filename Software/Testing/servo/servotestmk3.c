#include "stdlib.h"
#include "avr/io.h"
#include "util/delay.h"

#define CPU_SPEED 2000000

#define compareconst 1600
#define comparemax 4500
#define comparechange 5
#define cycles 25

#define TC_GetOverflowFlag( _tc ) ( (_tc)->INTFLAGS & TC0_OVFIF_bm )
#define TC_ClearOverflowFlag( _tc ) ( (_tc)->INTFLAGS = TC0_OVFIF_bm )
void PWMsetup(void)
{
	CLK.PSCTRL = 0x00; //no peripheral clock division?
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCCEN_bm | TC0_CCAEN_bm;
	TCD0.PER = 40000;
	PORTD.DIR = 0x05; //Set PC0 and PC2 to output
	
}

int main(void)
{
	int i;
	int delaycount;
	PORTF.DIR = 0x01;

	PWMsetup();

	while(1)
	{
		i = 2600;
		TCD0.CCA = i;
		TCD0.CCC = i;
		for(delaycount = 0; delaycount <= cycles; delaycount++)
		{
			while(0 == TC_GetOverflowFlag( &TCD0 )){}
			TC_ClearOverflowFlag( &TCD0 );
		}
		i = i + 100;
		TCD0.CCA = i;
		TCD0.CCC = i;
		for(delaycount = 0; delaycount <= cycles; delaycount++)
		{
			while(0 == TC_GetOverflowFlag( &TCD0 )){}
			TC_ClearOverflowFlag( &TCD0 );
		}
		i = i - 200;
		TCD0.CCA = i;
		TCD0.CCC = i;
		for(delaycount = 0; delaycount <= cycles; delaycount++)
		{
			while(0 == TC_GetOverflowFlag( &TCD0 )){}
			TC_ClearOverflowFlag( &TCD0 );
		}
	}
	return 0;
}


