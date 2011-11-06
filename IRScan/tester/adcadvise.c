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
	int bufferZero, bufferOne;
	char xbeebuffer[20];
	int historyZero[10], historyOne[10];
	char data;
	int i = 0;
	int j;
	int slopeZero = 0, slopeOne=0;
	for(j = 0; j < 10; j ++){
		historyZero[j] = 0;
		historyOne[j] = 0;
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
			sendchar(data);
			if(data == 'r'){
				//PORTD ^= 0x20;
				bufferZero = readadc(0);
				historyZero[i] = bufferZero;
				bufferOne = readadc(1);
				historyOne[i] = bufferOne;	
				i ++;
				if(i == 10){
					i = 0;
				}
				for(j = 0; j < 9; j ++){
					slopeZero += (historyZero[i] - historyZero[i + 1])/2;
					slopeOne += (historyOne[i] - historyOne[i + 1])/2;
				}
				sprintf(xbeebuffer, "%3d %3d\n\r %3d %3d\n\r", slopeZero, bufferZero,slopeOne, bufferOne);
				//sprintf(xbeebuffer, " %d \n\r", buffer);
				sendstring(xbeebuffer);
				slopeZero = slopeOne = 0;
			}
		}
	}
	return 0;
}
				
	
