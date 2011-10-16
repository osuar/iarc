#include <stdio.h>
#include "avr_compiler.h"
#include "usart_driver.h"
#include "twi_master_driver.h"
#include "support.h"


#define AVR_ENTER_CRITICAL_REGION( ) uint8_t volatile saved_sreg = SREG; \
                                     cli();

#define AVR_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;

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


USART_data_t xbee;
USART_data_t IMU;

int main(void){

	PORTE.DIR |= 0b00001000;
	PORTC.DIR |= 0b00001000;
	PORTF.DIR |= 0x03; 

	
	char receivexbee = 0;
	char receive[100];
	int j = 0;
	int bytetobuffer = 0;

	char imuread = 0xD2;
	char xbeebuffer[100];
	int sendflag = 0;

	char floatreceiveaccel[3];
	char floatreceiveang[3];

	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc; 
	TCC0.PER = 40000; 
	
	/**Setup Xbee*/
	USART_InterruptDriver_Initialize(&xbee, &USARTC0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTC0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);

	/**Setup IMU*/
	USART_InterruptDriver_Initialize(&IMU, &USARTE0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(IMU.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(IMU.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 19 , -4);
	USARTE0.CTRLB |= 0x04;
	USART_Rx_Enable(IMU.usart);
	USART_Tx_Enable(IMU.usart);

	while(1){
		if(USART_RXBufferData_Available(&xbee)){
			receivexbee = USART_RXBuffer_GetByte(&xbee);
			PORTF.OUT ^= 0x01;
			if(receivexbee == 'r'){
				do{
					bytetobuffer = USART_TXBuffer_PutByte(&IMU, imuread);
				}while(!bytetobuffer);
			}
		}
/**
		if(USART_RXBufferData_Available(&IMU)){
			PORTF.OUT ^= 0x02;
			receive[42] = USART_RXBuffer_GetByte(&IMU);		
			for(j = 1; j < 43; j ++){
				while(!USART_RXBufferData_Available(&IMU));
				receive[42 - j] = USART_RXBuffer_GetByte(&IMU);
			}
			for(j = 0; j < 3; j ++){
				floatreceiveaccel[j] = output(&receive[38 - (4 * j )], 0b01111000);
				floatreceiveang[j] = output(&receive[26 - (4 * j)], 0b01111000);
			}
			
			sprintf(xbeebuffer, "  x %3i y %3i z %3i X %3i Y %3i Z %3i\n\r", floatreceiveaccel[0], floatreceiveaccel[1], floatreceiveaccel[2], floatreceiveang[0], floatreceiveang[1], floatreceiveang[2]);
			
			for(j = 0; xbeebuffer[j] != 0 ; j ++){
				do{
				bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[j]);
				}while(!bytetobuffer);
			}
		}
**/
	}
}
	



ISR(USARTE0_RXC_vect){
	USART_RXComplete(&IMU);
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&IMU);
}

ISR(USARTC0_RXC_vect){
	USART_RXComplete(&xbee);
}

ISR(USARTC0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
