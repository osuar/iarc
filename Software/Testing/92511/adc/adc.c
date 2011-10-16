#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include "avr_compiler.h"

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

int main(void){
	ADMUX = 0b01000000;
	ADCSRA = 0x80;
	int buffer;
	char xbeebuffer[10];

	UCSR0A = 0x02;
	UCSR0B = 0x18;
	UCSR0C = 0x06;
	UBRR0L = 12;

	DDRD = 0b00100010;
	

	while(1){
		PORTD ^= 0b00100000;
		buffer = readadc(0);
		sprintf(xbeebuffer, "  %d\n\r ", buffer);
		sendstring(xbeebuffer);
		_delay_ms(500);
	}
	return 0;
}
				
	
