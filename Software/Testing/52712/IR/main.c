#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr_compiler.h>
#include "usart.h"
#include "servo.h"
#include "adc.h"
#include <util/delay.h>

#define SONARCHANNEL 0
#define IRACHANNEL 1
#define IRBCHANNEL 2

int main(void)
{
	//initialize
	setup_usart();
	setup_adc();
	setup_servo(2);

	//initialize TIMER2. if works: modualize into diff file.
	TCCR2A = 0; //normal mode
	TCCR2B = 0b00000100; // clk/64 ~= 61Hz	

	unsigned int sonarData = 0;
	unsigned int irAData = 0;
	unsigned int irBData = 0;
	unsigned int servoPos = 64;
	unsigned int servoAngle = 0; 
	int servoDirection = 0;
	int i = 0;

	while(1)	//check for data request as fast as possible
{
		if(mygetchar() == 'r')
		{
			sendchar('a');
			sendchar((char) (sonarData >> 8));	//sonar HIGH byte
			sendchar((char) (sonarData & 0xFF));	//sonar LOW byte
			sendchar((char) (irAData >> 8));	//irA HIGH byte
			sendchar((char) (irAData & 0xFF));	//irA LOW byte
			sendchar((char) (irBData >> 8));	//irB HIGH byte	
			sendchar((char) (irBData & 0xFF));	//irB LOW byte
			sendchar((char) (servoAngle >> 8));	//servoPos HIGH byte
			sendchar((char) (servoAngle & 0xFF));	//servoPos LOW byte
			sendchar('z');
//			sendchar(10);
//			sendchar(13);
//			_delay_ms(500);
		}
		
		if(TIFR2 & 0x01)	//if timer has elapsed
		{
			TIFR2 &= 0b1111110; //reset timer
			sonarData = readadc(SONARCHANNEL);
			irAData = readadc(IRACHANNEL);
			irBData = readadc(IRBCHANNEL);
			
			if(i >= 1)
			{
				i = 0;
				
				if(servoDirection)	servoPos--;
				else			servoPos++;

				if(servoPos >= 199)	servoDirection = 1;
				else if(servoPos <= 64)	servoDirection = 0;

				move_servo(servoPos, 1);
				move_servo(servoPos, 2);

				servoAngle = ((servoPos - 64)*(2/3)); 
			}
			i++;
		}

	}
}
