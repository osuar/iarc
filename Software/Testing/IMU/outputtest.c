#include "avr/io.h"

int main( void )
{
	PORTC.DIR = 0xFF; //set all pins in portC to output
	PORTC.OUT = 0xFF; //set all pins in portC to high
	return 0;
}

