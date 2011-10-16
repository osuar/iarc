/**
@author Daniel Sidlauskas Miller

Source code for flight control support functions
*/

#include <stdlib.h>
#include "avr_compiler.h"
#include "support.h"
#include "usart_driver.h"


void ValueFunk(int accelx, int accely, int accelz, int rollx, int rolly, int rollz, short int *servol, short int *servor, short int *motorl, short int *motorr){
	
	*servol = SERVOINI;
	*servol += SERVOLAX * accelx;
	*servol += SERVOLAY * accely;
	*servol += SERVOLAZ * accelz;
	*servol += SERVOLRX * rollx;
	*servol += SERVOLRY * rolly;
	*servol += SERVOLRZ * rollz;

	*servor = SERVOINI;
	*servor += SERVORAX * accelx;
	*servor += SERVORAY * accely;
	*servor += SERVORAZ * accelz;
	*servor += SERVORRX * rollx;
	*servor += SERVORRY * rolly;
	*servor += SERVORRZ * rollz;

	*motorl = MOTORLNORM;
	*motorl += MOTORLAX * accelx;
	*motorl += MOTORLAY * accely;
	*motorl += MOTORLAZ * accelz;
	*motorl += MOTORLRX * rollx;
	*motorl += MOTORLRY * rolly;
	*motorl += MOTORLRZ * rollz;

	*motorr = MOTORRNORM;
	*motorr += MOTORRAX * accelx;
	*motorr += MOTORRAY * accely;
	*motorr += MOTORRAZ * accelz;
	*motorr += MOTORRRX * rollx;
	*motorr += MOTORRRY * rolly;
	*motorr += MOTORRRZ * rollz;
}

