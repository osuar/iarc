#include <stdlib.h>
#include "../drivers/avr_compiler.h"
#include <stdio.h>

int main(void){
	PORTC.DIR = 1;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm;
	TCC0.PER = 20000;

	TCC0.CCA = 2800;
}

