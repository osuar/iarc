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
	char buffer0;
	char buffer1;
	char xbeebuffer[20];
	char history0[10];
	char history1[10];
	char data;
	int i = 0;
	int j;
	char slope0 = 0;
	char slope1 = 0;
	for(j = 0; j < 10; j ++){
		history0[j] = 0;
		history1[j] = 0;
	}

	UCSR0A = 0x02;
	UCSR0B = 0x18;
	UCSR0C = 0x06;
	UBRR0L = 12;

	DDRD = 0b10110010;
	PORTD = 0b00110000;
	

	while(1){
		if(UCSR0A & 0x80){
			data = UDR0;
			//sendchar(data);
			if(data == 'r'){
				//PORTD ^= 0x20;
				buffer0 = readadc(0)/10;
				buffer1 = readadc(2)/10;
				
				history0[i] = buffer0;
				history1[i] = buffer1;
				
				i ++;
				if(i == 10){
					i = 0;
				}
				for(j = 0; j < 9; j ++){
					slope0 += (history0[i] - history0[i + 1])/2;
					slope1 += (history1[i] - history1[i + 1])/2;
				}
				//sprintf(xbeebuffer, "%4d %4d %4d %4d\n\r", slope0, buffer0, slope1, buffer1);
				
				//sprintf(xbeebuffer, " %d \n\r", buffer);
				xbeebuffer[0] = 'r';
				xbeebuffer[1] = ' ';
				xbeebuffer[2] = buffer0;
				xbeebuffer[3] = buffer1;
				xbeebuffer[4] = 0;
				sendstring(xbeebuffer);
				slope0 = 0;
				slope1 = 0;
			}
		}
	}
	return 0;
}
				
	
