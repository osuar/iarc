#include <stdlib.h>
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "usart_driver.h"
#include "support.h"
#include <stdio.h>
#include <stddef.h>
#include "adc.h"

USART_data_t xbee;
volatile char input;
volatile char readdata = 0;


int main(void){
	char xbeebuffer[100];
	int adcSample;

	/**Setup Xbee*/
	PORTD.DIR = 0b00001000;
	PORTF.DIR = 3;

	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
		PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	USART_InterruptDriver_Initialize(&xbee, &USARTD0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTD0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);


	ADC_Ch_InputMode_and_Gain_Config(&ADC_BK.CH0, ADC_CH_INPUTMODE_DIFF_gc, ADC_DRIVER_CH_GAIN_NONE); 	// differential mode, no gain
	ADC_Ch_InputMux_Config(&ADC_BK.CH0, pin, ADC_CH_MUXNEG_PIN1_gc);		

	ADC_Reference_Config(&ADC_BK, ADC_REFSEL_VCC_gc); 		// use Vcc/1.6 as ADC reference

	ADC_ConvMode_and_Resolution_Config(&ADC_BK, ADC_ConvMode_Signed, ADC_RESOLUTION_12BIT_gc);

	ADC_Prescaler_Config(&ADC_BK, ADC_PRESCALER_DIV32_gc);

	while(1){
		if(readdata){
			readdata = 0;
			if(input == 'r'){
				adc_start_conversion(&ADCA, ADC_CH0);
				adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
				adcSample = adcch_get_signed_result(&ADCA, 0);
				sprintf(xbeebuffer, " %d\n\r", adcSample);
				sendstring(&xbee, xbeebuffer);



			}
		}
	}
}

				
ISR(USARTD0_RXC_vect){
	PORTF.OUT = 1;
	USART_RXComplete(&xbee);
	input = USART_RXBuffer_GetByte(&xbee);
	readdata = 1;
}

ISR(USARTD0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
