#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr_compiler.h>
#include "usart.h"
#include "servo.h"
#include "adc.h"
#include <util/delay.h>

#define ADCCHANNEL 0
#define RUNNINGAVGWEIGHT 0.4	//Higher = faster response, more spikes. Lower = slower response, less spikes.

int main(void)
{
	//initialize
	setup_usart();
	setup_adc();
	//setup_servo(1);

	//initialize TIMER2. if works: modualize into diff file.
	TCCR2A = 0; //normal mode
	TCCR2B = 0b00000100; // clk/64 ~= 61Hz	

	int adcDataPrev = 0;
	int adcData = 0;
	int adcDataDeriv = 0;
	int servoPos = 0; 
	char buffer[50];
	int i = 0;

	while(1)	//check for data request as fast as possible
{
		if(mygetchar() == 'r')
		{
			sendchar('a');
			sendchar((char) (adcData >> 8));
			sendchar((char) adcData & 0xFF);
			sendchar('z');
//			sendchar(10);
//			sendchar(13);
//			_delay_ms(500);
		}
		
		if(TIFR2 & 0x01)	//if timer has elapsed
		{
			TIFR2 &= 0b1111110; //reset timer
			
//			adcDataPrev = adcData;
			adcData = readadc(ADCCHANNEL);
//			adcDataDeriv = adcData - adcDataPrev;
//			sprintf(buffer, "ADC Data: %d", adcData);
//			sendstring(buffer);	
//			sendchar('\r');
//			sendchar('\n');
		}

	}
}
