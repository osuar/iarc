#include "stdlib.h"
#include "avr/io.h"
#include "util/delay.h"

//#define CPU_SPEED 2000000

#define TC_GetOverflowFlag( _tc ) ( (_tc)->INTFLAGS & TC0_OVFIF_bm )
#define TC_ClearOverflowFlag( _tc ) ( (_tc)->INTFLAGS = TC0_OVFIF_bm )

void PWMsetup(void)
{
	CLK.PSCTRL = 0x00; //no peripheral clock division?
	TCD0.CTRLA = TC_CLKSEL_DIV1024_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 625;
	PORTD.DIR = 0x0A; //Set PC1 and PC3 to output
	
}

int main(void)
{
	int i; //i is variable compare value for pwm
	int delaycount;
	Config32MHzClock();
	PWMsetup();
	

	for(i = 31;i <= 48; i+=1){
		TCD0.CCB = i + 2;
		TCD0.CCD = i;
		
		for(delaycount = 0; delaycount <= 50; delaycount ++)
		{
			//make sure change isn't funky, wait for loop to complete
			while( TC_GetOverflowFlag( &TCD0 ) == 0 ){}

			// Clear overflow flag.
			TC_ClearOverflowFlag( &TCD0 );
		}

	}
	return 0;

}


