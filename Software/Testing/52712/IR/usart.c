#include <avr/io.h>
#include "avr_compiler.h"
#include "usart.h"

void setup_usart(void)
{
        DDRD |= 0x02; // set the transmitt pin to be an output
        UCSR0A = 0x02; // double transmition speed
        UCSR0B = 0x18; // recieve enable, transmit enabled
        UCSR0C = 0x06; // 8 bit character selected 
        UBRR0L = 12;   // baud rate UBRR0L = (F_CLK /(16*BAUD))*2 - 1 BUAD 9600
}

void sendchar(char input)
{
        while(!(UCSR0A & 0x20));
	UDR0 = input;
}

void sendstring(char* string)
{
        int i;
        for(i = 0; string[i] != 0; i ++)
        {
                sendchar(string[i]);
        }
}

char mygetchar(void)
{
	if((UCSR0A & 0x80)){
		return UDR0;
	}
	return 0;
}

