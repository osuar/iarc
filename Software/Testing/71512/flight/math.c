#include <stdlib.h>
#include "../../drivers/avr_compiler.h"
#include "math.h"

int abs(int i){
	if(i < 0) {
		i *= -1;
		return i;
	}
	else {
		return i;
	}
}

/* Code taken from Craig McQueen*/
uint32_t mySqrt(uint32_t input){
	uint32_t op  = input;

	uint32_t res = 0;
	uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type


	// "one" starts at the highest power of four <= than the argument.
	while (one > op)
	{
		one >>= 2;
	}

	while (one != 0)
	{
		if (op >= res + one)
		{
			op = op - (res + one);
			res = res +  2 * one;
		}
		res >>= 1;
		one >>= 2;
	}

	/* Do arithmetic rounding to nearest integer */
	if (op > res)
	{
		res++;
	}

	return res;
}


