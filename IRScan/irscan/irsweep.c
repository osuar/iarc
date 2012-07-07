#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include "avr_compiler.h"
#include <math.h>


#define STEPS 10 // number of steps in a full sweep of the servo
#define NUM_SERVOS 1 //number of servos in use
#define INCREMENT 25 //change in the duty cycle needed to ellicit a one increment step
#define LENGTH 25 // this is the number of blocks of data that can be held
// AREF pin needs external capacitor for opertaion of ADC

typedef int array_1[NUM_SERVOS];
typedef int array_2[STEPS][NUM_SERVOS];
typedef int array_3[LENGTH][STEPS][NUM_SERVOS];

void setup_servo(int num) // Sets up pwm output for a "num" number of servos(max of 2)
{
	switch(num)
	{
		case 1:
			DDRD |= 0b01000000;     // Set PD6(OC0A) Output
			TCCR0A = 0b10000011;    // Fast PWM mode, Non-inverting mode  
			TCCR0B = 0b00000101;    // clk/1024 

			OCR0A = 128;            // Set Output Compare Register
			break;
		case 2:
			DDRD |= 0b01100000;     // Set PD6(OC0A) Output Set PD5(OC0B) Output
			TCCR0A = 0b11000011;    // Fast PWM mode, Non-inverting mode  
			TCCR0B = 0b00000101;    // clk/1024 

			OCR0A = 128;            // Set Output Compare Register
			OCR0B = 128;
			break;
	}
}

void sweep(array_2 *data) // takes a 3 dimensional array, sweeps back and forth
{                         // moving servo first up, then down, and reading for each value
			  // reads a "LENGHT" number of sweeps before returning
	int i, j, k;
	for(i = 0; i < LENGTH; i++)
	{
		for(j = 0; j < STEPS; j++)
		{
			for(k = 0; k < NUM_SERVOS; k++)
			{
				data[i][j][k] = readadc(k+=1);
				move_servo(k+=1, pow(-1, i));
			}
		}
	}
}

void move_servo(int pin, int direction)
{
	if(direction == 1)
	{
		switch(pin)
		{
			case 1:
				OCR0A += INCREMENT;
			case 2:
				OCR0B += INCREMENT;
		}
	}

	else
	{	
		switch(pin)
		{
			case 1:
				OCR0A -= INCREMENT;
			case 2:
				OCR0B -= INCREMENT;
		}
	}
} 

int readadc(int channel)
{
	int output;
	ADMUX |= channel; // this selects the register to be read (ADC0 - ADC7)
	ADCSRA |= 0x40;   // start the conversion
	while(!(ADCSRA & 0x10)); // should be interrupt, delay until sample is ready
	output = ADCL + (256 * ADCH); //ADCH hold values above the 8 bit values, therefore must be multiplied by 256
	ADMUX &= ~(0x0F); //clears the selection register
	return output; // returns the raw ADC inteteger
}

void setup_adc(void)
{
	ADMUX = 0b01000000; //use vcc as compare voltage, external cap on AREF pin
	ADCSRA = 0x80; //this enables the ADC 
}
	
void sendchar(char input)
{
	while(!(UCSR0A & 0x20));
	UDR0 = input;
}
void sendstring(char *string)
{
	int i;
	for(i = 0; string[i] != 0; i ++)
	{
		sendchar(string[i]);
	}
}
	

int main(void)
{

	setup_adc();
	setup_servo(1);
	array_3 data_stor;

	int i, j, k = 0;
	for(i = 0; i < LENGTH; i++)
	{
		for(j = 0; j < STEPS; j++)
		{
			for(k = 0; k < NUM_SERVOS; k++)
			{
				data_stor[i][j][k] = 0;
			}
		}
	}
	while(1)
	{
		sweep(data_stor);
	}

	DDRD = 0b10110010;
	PORTD = 0b00110000;

	return 0;
}
				
	
