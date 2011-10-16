#include "stdlib.h"
#include "avr/io.h"

void PWMsetup(void)
{
	CLK.PSCTRL = 0x00; //no peripheral clock division?
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCCEN_bm | TC0_CCAEN_bm;
	TCD0.PER = 40000;
	PORTC.DIR = 0x05; //Set PC0 and PC2 to output
	
}

void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL |= OSC_RC32MEN_bm; // enable 32MHz oscillators
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = 0x01; //select sysclock 32MHz osc
}
