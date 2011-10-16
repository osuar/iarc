/**
@author Daniel Sidlauskas Miller

Source code for flight control support functions
*/

#include <stdlib.h>
#include "avr_compiler.h"
#include "support.h"
#include "usart_driver.h"
#include <stdio.h>

void CCPWrite( volatile uint8_t * address, uint8_t value )
{
	#if defined __GNUC__
	AVR_ENTER_CRITICAL_REGION( );
	volatile uint8_t * tmpAddr = address;
	#ifdef RAMPZ
	RAMPZ = 0;
	#endif
	asm volatile(
		"movw r30,  %0"      "\n\t"
		"ldi  r16,  %2"      "\n\t"
		"out   %3, r16"      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
		: "r16", "r30", "r31"
	);

	AVR_LEAVE_CRITICAL_REGION( );
	#endif
}



unsigned char exponent(char* bytes){
	unsigned char exp = 0;
	int i;
	int k;
	int j;
	for(i = 0; i < 7; i ++){
		j = 1;
		for(k = 5; k > i; k --){
			j *= 2;
		}
		if(bytes[3] & j){
			exp |= 2 * j;
		}
	}
	if(bytes[2] & 0x80){
		exp |= 1;
	}
	printf("%i\n", exp);
	return exp;
}

char output(char* bytes, unsigned char min){
	char realout = 0;
	int i;
	int j;
	int p;
	int k;
	unsigned char exp = exponent(bytes)- min;
	realout = 1 << exp;
	for(i = exp, p = 0; i > 0; i --, p ++){
		j = 1;
		for(k = 6; k > p; k --){
			j *= 2;
		}
		if(bytes[2] & j){
			realout |= 1 << (i - 1);
		}
	}
	if(bytes[3] & 0x80){
		realout *= -1;
	}
	printf("%i\n", realout);
	return realout;
}

unsigned char outputlong(char* bytes, unsigned char min){
	unsigned char realout = 0;
	int i;
	int j;
	int p;
	int k;
	unsigned char exp = exponent(bytes) - min;
	realout = 1 << exp;
	for(i = exp, p = 0; i > 0; i --, p ++){
		j = 1;
		for(k = 6; k > p; k --){
			j *= 2;
		}
		if(bytes[2] & j){
			realout |= 1 << (i - 1);
		}
	}
	printf("%i\n", realout);
	return realout;
}

void ValueFunk(int accelx, int accely, int accelz, int rollx, int rolly, int rollz, short int *servol, short int *servor, short int *motorl, short int *motorr){
	
	*servol = SERVOLINI;
	*servol += SERVOLAX * accelx;
	*servol += SERVOLAY * accely;
	*servol += SERVOLAZ * accelz;
	*servol += SERVOLRX * rollx;
	*servol += SERVOLRY * rolly;
	*servol += SERVOLRZ * rollz;
	if(*servol >= MAXSERVO){
		*servol = MAXSERVO;
	}
	if(*servol <= MINSERVO){
		*servol = MINSERVO;
	}

	*servor = SERVORINI;
	*servor += SERVORAX * accelx;
	*servor += SERVORAY * accely;
	*servor += SERVORAZ * accelz;
	*servor += SERVORRX * rollx;
	*servor += SERVORRY * rolly;
	*servor += SERVORRZ * rollz;
	if(*servor >= MAXSERVO){
		*servor = MAXSERVO;
	}
	if(*servor <= MINSERVO){
		*servor = MINSERVO;
	}

	*motorl = MOTORLNORM;
	*motorl += MOTORLAX * accelx;
	*motorl += MOTORLAY * accely;
	*motorl += MOTORLAZ * accelz;
	*motorl += MOTORLRX * rollx;
	*motorl += MOTORLRY * rolly;
	*motorl += MOTORLRZ * rollz;
	if(*motorl >= MAXMOTOR){
		*motorl = MAXMOTOR;
	}
	if(*motorl <= MINMOTOR){
		*motorl = MINMOTOR;
	}

	*motorr = MOTORRNORM;
	*motorr += MOTORRAX * accelx;
	*motorr += MOTORRAY * accely;
	*motorr += MOTORRAZ * accelz;
	*motorr += MOTORRRX * rollx;
	*motorr += MOTORRRY * rolly;
	*motorr += MOTORRRZ * rollz;
	if(*motorr >= MAXMOTOR){
		*motorr = MAXMOTOR;
	}
	if(*motorr <= MINMOTOR){
		*motorr = MINMOTOR;
	}

}
/*----------------------------------------------------------------------

 * TestByteOrder()

 * Tests byte alignment to determine Endian Format of local host.

 *

 * returns:     The ENDIAN platform identifier.

 *--------------------------------------------------------------------*/

int TestByteOrder()

{

   short int word = 0x0001;

   char *byte = (char *) &word;

   return(byte[0] ? LITTLE_ENDIAN : BIG_ENDIAN);

}



/*----------------------------------------------------------------------

 * FloatFromBytes

 * Converts bytes to Float.

 *

 * parameters:  pBytes : received buffer containing pointer to 4 bytes

 *

 * returns:     a float value.

 *--------------------------------------------------------------------*/

float FloatFromBytes(const unsigned char* pBytes)

{

	union{
		char bytes[4];
		float f;
	}u;
	int i;
	for(i = 0; i < 4; i ++){
		u.bytes[3 - i] = pBytes[i];
	}	
	

	return u.f; 

}
