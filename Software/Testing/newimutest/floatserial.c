#include <stdio.h>
#include <stdlib.h>



void binaryfunk(unsigned char *bits){
	unsigned char sign = (bits[3] & (0x80)) >> 7;
	sign += 48;
	unsigned char exponent[8];
	unsigned char mantissa[23];
	int i;
	int j;
	int k;
	int p;
	unsigned char realexp = 0;	
	char realout = 0;

	for(i = 0; i < 7; i ++){
		j = 1;
		for(k = 6; k > i; k --){
			j *= 2;
		}
		exponent[i] = (bits[3] & (j)) / j;
		exponent[i] += 48;
		if(exponent[i] == 49){
			realexp |= 2 * j;
		}
	}
	exponent[7] = (bits[2] & 0x80) >> 7;
	exponent[7] += 48;
	if(exponent[7] == 49){
		realexp |= 0x01;
	}

	realexp -= 0b01111000;

	for(i = 0; i < 7; i++){                                                //fills mantissa
		j = 1;
		for(k = 6; k > i; k --){
			j *= 2;
		}
                mantissa[i] = ((bits[2] & (j)) / j);
		mantissa[i] += 48;
        }
        for(i = 7; i < 15; i++){
                mantissa[i] = ((bits[1] & ((14 - i) << 1)) >> (14 - i));
		mantissa[i] += 48;
        }
        for(i = 15; i < 23; i++){
                mantissa[i] = (bits[0] & ((22 - i) << 1)) >> (22 - i);
		mantissa[i] += 48;
        }

	j = 1;
	for(k = 0; k < realexp; k ++){
		j *= 2;
	}
	realout |= j;

	for(i = realexp, p = 0; i > 0; i--, p++){
		if(mantissa[p] == 49){
		j = 1;
		for(k = 0; k < i - 1; k ++){
			j *= 2;
		}
		realout |= j;
		}
	}

	if(sign == 49){
		realout *= -1;
	}

	printf("sign %c	exponent %s	mantissa %s\n", sign, exponent, mantissa);
	printf("%i %i\n",realexp, realout);

	
        
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

char output(unsigned char* exp, char* bytes, unsigned char min){
	char realout = 0;
	int i;
	int j;
	int p;
	int k;
	unsigned char exponent = *exp - min;
	realout = 1 << exponent;
	for(i = exponent, p = 0; i > 0; i --, p ++){
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

unsigned char outputlong(unsigned char* exp, char* bytes, unsigned char min){
	unsigned char realout = 0;
	int i;
	int j;
	int p;
	int k;
	unsigned char exponent = *exp - min;
	realout = 1 << exponent;
	for(i = exponent, p = 0; i > 0; i --, p ++){
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

int main(void){
	
	unsigned char bytes[10];
	FILE *input;
	input = fopen("/dev/ttyUSB0", "r+");
	int i;
	unsigned char exp;

	union{
		char imu[5];
		float output;
	}u;
		
	if(!input){
		printf("File not found");
		return 1;
	}

	fputc('r', input);

	while(1){
		
		if(!feof(input)){
			printf("got something\n");
			bytes[0] = fgetc(input);
			if(bytes[0] == 0x20){
				while(feof(input)){
					printf("circling\n");
				}
				bytes[1] = fgetc(input);
				if(bytes[1] == 0x20){
					for(i = 0; i < 7; i ++){
						while(feof(input));
						bytes[i + 2] = fgetc(input);
					}
					for(i = 0; i < 4; i ++){
						u.imu[3 - i] = bytes[i + 2];
						printf(" ");
					}
					printf("%f\n", u.output);
					exp = exponent(u.imu);
					output(&exp, u.imu, 0b01111000);
					outputlong(&exp, u.imu, 0b01111000);
				}
			}
		}
	
		sleep(1);
		printf("trying again\n");
		fputc('r', input);
		
	}

	return 0;
}
			
