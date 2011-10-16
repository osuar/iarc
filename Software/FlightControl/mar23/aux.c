/**
@author Daniel Sidlauskas Miller

Source code for flight control support functions
*/

#include <stdlib.h>
#include "avr_compiler.h"
#include "aux.h"

//Servo L new value calculator
int ServoLValueFunk(int accelx, int accely, int accelz)
{
	int servolvalue = SERVOINI;
	servolvalue += SERVOLAX * accelx;
	servolvalue += SERVOLAY * accely;
	servolvalue += SERVOLAZ * accelz;
	return servolvalue;
}

//Servo R new value calculator
int ServoRValueFunk(int accelx, int accely, int accelz)
{
	int servorvalue = SERVOINI;
	servorvalue += SERVORAX * accelx;
	servorvalue += SERVORAY * accely;
	servorvalue += SERVORAZ * accelz;
	return servorvalue;
}

int MotorLValueFunk(int accelx, int accely, int accelz)
{
	int motorlvalue = MOTORNORM;
	motorlvalue += MOTORLAX * accelx;
	motorlvalue += MOTORLAY * accely;
	motorlvalue += MOTORLAZ * accelz;
	return motorlvalue;
}

int MotorRValueFunk(int accelx, int accely, int accelz)
{
	int motorrvalue = MOTORNORM;
	motorrvalue += MOTORRAX * accelx;
	motorrvalue += MOTORRAY * accely;
	motorrvalue += MOTORRAZ * accelz;
	return motorrvalue;
}
