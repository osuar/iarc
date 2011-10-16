#define F_CPU		2000000
#define CPU_PRESCALER	1

#include	"TC_driver.h"
#include	"avr_compiler.h"
#include	"stdlib.h"
#include	"util/delay.h"

int main( void )
{
	void PWMsetup();

	uint16_t compareValue = 0x0004;	

	TC_SetCompareA( &TCC0 , compareValue);

	return 0;
}

void PWMsetup( void )
{
	PORTC.DIR = 0x01; //Sets PC0 to output

	TC_SetPeriod( &TCC0 , 0x0020 ); //Sets count period to 32

	TC0_ConfigWGM( &TCC0 , TC_WGMODE_SS_gc ); //Set single slope

	TC0_EnableCCChannels( &TCC0 , TC0_CCAEN_bm ); //Set CC A to output

	TC0_ConfigClockSource( &TCC0 , TC_CLKSEL_DIV1024_gc ); //Set clock to speed/1024

}


