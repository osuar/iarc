/**
@author Daniel Sidlauskas Miller

Source code for flight control support functions
*/

#include <stdlib.h>
#include "../../drivers/avr_compiler.h"
#include "../../drivers/usart_driver.h"
#include "../../drivers/twi_master_driver.h"
#include "support.h"
#include <stdio.h>


void sendchar(USART_data_t * uart, char buffer){
	char bytetobuffer;
	do{
		bytetobuffer = USART_TXBuffer_PutByte(uart, buffer);
	}while(!bytetobuffer);

}
void sendstring(USART_data_t * uart, char *buffer){
	int i;
	for(i=0;buffer[i]!=0;i++){
		sendchar(uart, buffer[i]);
	}
}



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



void sprintBinary(char * buffer, int * value){
	int i;
	int k = 2;
	buffer[0] = '0';
	buffer[1] = 'b';
	for(i = 15; i >= 0; i --, k++){
		if(*value & (1 << i)){
			buffer[k] = '1';
		}
		else{
			buffer[k] = '0';
		}
	}
	sprintf(&buffer[18], "\n\r");
}
			
/*TWI Master Initiate*/
void twiInitiate(TWI_Master_t *title,TWI_t *interface){
	TWI_MasterInit(title, interface, TWI_MASTER_INTLVL_HI_gc, TWI_BAUDSETTING);
	interface->MASTER.CTRLB |= 0x04;
}


/*USART Initiate*/
void uartInitiate(USART_data_t * title,USART_t * interface){
	USART_InterruptDriver_Initialize(title, interface, USART_DREINTLVL_LO_gc);
	USART_Format_Set(title->usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(title->usart, USART_RXCINTLVL_LO_gc);
	USART_Baudrate_Set(interface, 22 , -2);
	title->usart->CTRLB |= 0x04;
	USART_Rx_Enable(title->usart);
	USART_Tx_Enable(title->usart);
	
}

void motorSpeed(int * pry,
		int * integration, 
		int * gyroint, 
		int * joystick, 
		int * motorSpeeds,
		int * pidValues,
		int * pidValuesDen){
	int i;
	int prybuffer[3];
	for(i=0;i<3;i++){
		prybuffer[i] = pry[i];
	}
	for(i=0;i<2;i++){
		if(prybuffer[i] > 200){
			prybuffer[i] = 200;
			}
		else if(prybuffer[i] < -200){
			prybuffer[i] = -200;
		}
	}
	for(i = 0; i < 4; i ++){
		motorSpeeds[i] = MOTORREG;
		//Joystick Throttle
		motorSpeeds[i] += joystick[2] * ZJOYSENS + joystick[4] * THROTTLEMAIN;
	}
	//For x axis rotating on Z
	motorSpeeds[0] += (prybuffer[1] * pidValues[0]/pidValuesDen[0]) + (gyroint[1] * pidValues[2]/pidValuesDen[2]);
	motorSpeeds[2] -= (prybuffer[1] * pidValues[0]/pidValuesDen[0]) + (gyroint[1] * pidValues[2]/pidValuesDen[2]);
	//Joystick X axis tilt
	motorSpeeds[0] -= joystick[1] * TILTJOYSENS;
	motorSpeeds[2] += joystick[1] * TILTJOYSENS;
	//Joystick Z axis rotate (slow down speed up both motors)
	if(joystick[3] > 0){
		motorSpeeds[0] += joystick[3] * ROTJOYSENSDOWN;
		motorSpeeds[2] += joystick[3] * ROTJOYSENSDOWN;
	}
	else{
		motorSpeeds[0] += joystick[3] * ROTJOYSENSUP;
		motorSpeeds[2] += joystick[3] * ROTJOYSENSUP;
	}
	
	//For y axis rotating on Z
	motorSpeeds[1] += (prybuffer[0] * pidValues[0]/pidValuesDen[0]) + (gyroint[0] * pidValues[2]/pidValuesDen[2]);
	motorSpeeds[3] -= (prybuffer[0] * pidValues[0]/pidValuesDen[0]) + (gyroint[0] * pidValues[2]/pidValuesDen[2]);
	//Joystick Y axis tilt
	motorSpeeds[1] -= joystick[0] * TILTJOYSENS;
	motorSpeeds[3] += joystick[0] * TILTJOYSENS;
	//Joystick Z axis rotate (slow down speed up both motors)
	if(joystick[3] < 0){
		motorSpeeds[1] -= joystick[3] * ROTJOYSENSDOWN;
		motorSpeeds[3] -= joystick[3] * ROTJOYSENSDOWN;
	}
	else{
		motorSpeeds[1] -= joystick[3] * ROTJOYSENSUP;
		motorSpeeds[3] -= joystick[3] * ROTJOYSENSUP;
	}


}
