/*
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include "avr_compiler.h"
#include <math.h>
*/

#include "all.h"


/*
#define STEPS 28 //number of steps in a full sweep of the servo
#define NUM_SERVOS 1 //number of servos in use
#define INCREMENT 1 //change in the duty cycle needed to ellicit a one increment step
#define LENGTH 2 // number of block of data that can be held
#define SERVO_SPEED 50 // this determines the ammount of delay between the movement of the servo
*/

int main(void)
{
	int dummy[STEPS];
	int deriv[STEPS];
	int i = 0;
	int c = 0;
	int count = 0;
	coord data[STEPS];
	coord coordinates[STEPS];

	//setups
	setup_usart();
	setup_adc();
	setup_servo(1);
	_delay_ms(1000); // delay for 1 seconds

	while(1)
	{
		sendchar('a');
		sweep(dummy); //gets the values
		derivitive(dummy, deriv); //finds the derivitive
		smooth(dummy, deriv); //based off of the deriv, smooth out dummy
		convert(dummy, coordinates);	
		average(data, coordinates, c); //take a running average, outputs via serial
		c++;
	}	
}
