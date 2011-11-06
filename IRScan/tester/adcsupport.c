#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include "avr_compiler.h"
#include "adcsupport.h"


int readadc(int channel){
	int output;
	ADMUX |= channel;
	ADCSRA |= 0x40; 
	while(!(ADCSRA & 0x10));
	output = ADCL + (256 * ADCH);
	ADMUX &= ~(0x0F);
	return output;
}
void sendchar(char input){
	while(!(UCSR0A & 0x20));
	UDR0 = input;
}
void sendstring(char *string){
	int i;
	for(i = 0; string[i] != 0; i ++){
		sendchar(string[i]);
	}
}
