#include <stdlib.h>
#include <avr/io.h>
#include "avr_compiler.h"
#include "servo.h"

void setup_servo(int num) // Sets up pwm output for a "num" number of servos(max of 2)
{
        switch(num)
        {
                case 1:
                        DDRD |= 0b01000000;     // Set PD6(OC0A) Output
                        TCCR0A = 0b10000011;    // Fast PWM mode, Non-inverting mode  
                        TCCR0B = 0b00000011;    // clk/256
                        OCR0A = 10;              // Set Output Compare Register
                        break;
                case 2:
                        DDRD |= 0b01100000;     // Set PD6(OC0A) Output Set PD5(OC0B) Output
                        TCCR0A = 0b10100011;    // Fast PWM mode, Non-inverting mode  
                        TCCR0B = 0b00000010;    // clk/8
                        OCR0A = 64;              // Set Output Compare Register
                        OCR0B = 64;
                        break;
        }
}



void move_servo(int pos, int servo) // move to the num postion
{
        while(TCNT0 != 0){}
	
	if(servo == 1)			OCR0A = pos;
	else if(servo == 2) 		OCR0B = pos;
}

