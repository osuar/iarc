#include <stdio.h>
#include <avr/io.h>

void Config32MHzClock(void);

int main(void)
{
  Config32MHzClock(); // configure sysclk=32MHz RC oscillator

  CLK.PSCTRL = 0x00;  // no division on peripheral clock

  // make clkout on PORTE:7
  PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PE7_gc;
  PORTE.DIR = (1<<7); // clkout

  // configure timer/counter0
  TCC0.CTRLA = 0x7;   // clk/1024

  // configure PORTF:0 as output to LED 
  PORTF.DIR=(1<<0);   // Eval-01 64A3 RevA, Eval-USB
//  PORTF.DIR=(1<<2);   // Eval-01 128A3,64A3 RevB

  // blink LED at 2Hz
  while(1)
  {
    PORTF.OUT ^= (1<<0);   // Eval-01 64A3 RevA, Eval-USB
//    PORTF.OUT ^= (1<<2);   // Eval-01 128A3, 64A3 RevB
    while(TCC0.CNT < 7812) // roughly 250ms
	asm("nop");
    TCC0.CNT=0;            // reset
  };

return 0;
};


void Config32MHzClock(void)
{
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  // initialize clock source to be 32MHz internal oscillator (no PLL)
  OSC.CTRL = OSC_RC32MEN_bm; // enable internal 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator ready
  CCP = CCP_IOREG_gc; //Security Signature to modify clock 
  CLK.CTRL = 0x01; //select sysclock 32MHz osc
};

