/**
@author Daniel Sidlauskas Miller

Source code for flight control support functions
*/

#include <stdlib.h>
#include "avr_compiler.h"
#include "support.h"

void ValueFunk(int accelx, int accely, int accelz, int rollx, int rolly, int rollz, volatile int *servol, volatile int *servor, volatile int *motorl, volatile int *motorr){
	
	*servol = SERVOINI;
	*servol += SERVOLAX * accelx;
	*servol += SERVOLAY * accely;
	*servol += SERVOLAZ * accelz;

	*servor = SERVOINI;
	*servor += SERVORAX * accelx;
	*servor += SERVORAY * accely;
	*servor += SERVORAZ * accelz;

	*motorl = MOTORLNORM;
	*motorl += MOTORLAX * accelx;
	*motorl += MOTORLAY * accely;
	*motorl += MOTORLAZ * accelz;

	*motorr = MOTORRNORM;
	*motorr += MOTORRAX * accelx;
	*motorr += MOTORRAY * accely;
	*motorr += MOTORRAZ * accelz;
}

