#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>

#define CPU_SPEED 2000000

int main( void )
{

	PORTF.DIR = 0x01;

	while(1)
	{
		if(0xff == 255)
		{
			PORTF.OUT ^= 0x01;
		}
		_delay_ms(100);
	}
}

