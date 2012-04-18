#include <avr/io.h>
#include "avr_compiler.h"
#include "adc.h"

void setup_adc(void)

{
        ADMUX = 0b01000000; //use vcc as compare voltage, external cap on AREF pin
        ADCSRA = 0x80; //this enables the ADC 
}

int readadc(int channel)
{
        int output, i;
        output = i = 0;
        for(i = 0; i < 15; i++)
        {
                ADMUX |= channel; // this selects the register to be read (ADC0 - ADC7)
                ADCSRA |= 0x40;   // start the conversion
                while(!(ADCSRA & 0x10)); // should be interrupt, delay until sample is ready
                output = output + ADCL + (256 * ADCH); //ADCH hold values above the 8 bit values, therefore must be multiplied by 256
                ADMUX &= ~(0x0F); //clears the selection register
        }
        return (output / 15); // returns the raw ADC inteteger
}

