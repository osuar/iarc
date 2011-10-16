#include <avr/io.h>
#include "avr_compiler.h"

int main(void){
	PORTF.DIR = 0x03;
	PORTF.OUT = 0x03;
	return 0;
}
