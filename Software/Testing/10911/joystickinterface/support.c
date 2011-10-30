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
		PORTF.OUT ^= 0x01;
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

char output(char* bytes, unsigned char min){
	char realout = 0;
	int i;
	int j;
	int p;
	int k;
	unsigned char exp = exponent(bytes)- min;
	realout = 1 << exp;
	for(i = exp, p = 0; i > 0; i --, p ++){
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

unsigned char outputlong(char* bytes, unsigned char min){
	unsigned char realout = 0;
	int i;
	int j;
	int p;
	int k;
	unsigned char exp = exponent(bytes) - min;
	realout = 1 << exp;
	for(i = exp, p = 0; i > 0; i --, p ++){
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

void ValueFunk(int accelx, int accely, int accelz, int rollx, int rolly, int rollz, short int *servol, short int *servor, short int *motorl, short int *motorr){
	
	accely += absval(accelx);
	accely += absval(accelz);

	*servol = SERVOLINI;
	*servol += SERVOLAX * accelx;
	*servol += SERVOLAY * accely;
	*servol += SERVOLAZ * accelz;
	*servol += SERVOLRX * rollx * absval(rollx);
	*servol += SERVOLRY * rolly * absval(rolly) / 8;
	*servol += SERVOLRZ * rollz;
	if(*servol >= MAXSERVO){
		*servol = MAXSERVO;
	}
	if(*servol <= MINSERVO){
		*servol = MINSERVO;
	}

	*servor = SERVORINI;
	*servor += SERVORAX * accelx;
	*servor += SERVORAY * accely;
	*servor += SERVORAZ * accelz;
	*servor += SERVORRX * rollx * absval(rollx);
	*servor += SERVORRY * rolly * absval(rolly) / 8;
	*servor += SERVORRZ * rollz;
	if(*servor >= MAXSERVO){
		*servor = MAXSERVO;
	}
	if(*servor <= MINSERVO){
		*servor = MINSERVO;
	}
	

	*motorl = MOTORLNORM;
	*motorl += MOTORLAX * accelx;
	*motorl += MOTORLAY * accely;
	*motorl += MOTORLAZ * accelz;
	*motorl += MOTORLRX * rollx;
	*motorl += MOTORLRY * rolly;
	*motorl += MOTORLRZ * rollz;
	if(*motorl >= MAXMOTOR){
		*motorl = MAXMOTOR;
	}
	if(*motorl <= MINMOTOR){
		*motorl = MINMOTOR;
	}

	*motorr = MOTORRNORM;
	*motorr += MOTORRAX * accelx;
	*motorr += MOTORRAY * accely;
	*motorr += MOTORRAZ * accelz;
	*motorr += MOTORRRX * rollx;
	*motorr += MOTORRRY* rolly;
	*motorr += MOTORRRY * rolly;
	*motorr += MOTORRRZ * rollz;
	if(*motorr >= MAXMOTOR){
		*motorr = MAXMOTOR;
	}
	if(*motorr <= MINMOTOR){
		*motorr = MINMOTOR;
	}

}

void getroll(char *rollcash, TWI_Master_t *imu, uint8_t *rollstartbyte){
	int i;
	do{
		while(imu->status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(imu, ROLL, rollstartbyte, 1, 10);
		while(imu->result == TWIM_RESULT_UNKNOWN);
	}while(!(imu->readData[0] & 0x01));
	for(i = 0; i < 5; i += 2){
		rollcash[i/2] += ((char)(imu->readData[i + 3]));
	}
}

void getaccel(int *accelcash, TWI_Master_t *imu, uint8_t *accelstartbyte){
	int i;
	do{
		while(imu->status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(imu, ACCEL, accelstartbyte, 1, 10);
		while(imu->result == TWIM_RESULT_UNKNOWN);
	}while(!(imu->readData[0] & 0x80));

	for(i = 0; i < 5; i += 2){
		if(imu->readData[i + 3] & 0x80){
			accelcash[i/2] -= 256 * (~imu->readData[i + 3] + 1);
			accelcash[i/2] -= (~imu->readData[i + 2] + 1);
		}
		else{
			accelcash[i/2] += 256 * imu->readData[i + 3];
			accelcash[i/2] += (imu->readData[i + 2]);
		}
	}
	for(i = 0; i < 3; i ++){
			accelcash[i] = accelcash[i] >> 2;
	}
}

char getoffset(int *acchisx,int * acchisy,int * acchisz,char * rolhisx,char * rolhisy,char * rolhisz,int * accelnorm,char * rollnorm){
	int lowest[6];
	int highest[6];
	int rolltotal[3] = {0,0,0};
	long long int acceltotal[3] = {0,0,0};
	int i;

/*
	for(i = 0; i < 3; i ++){
		accelnorm[i] = 0;
		rollnorm[i] = 0;
	}
*/

	highest[0] = acchisx[0];
	highest[1] = acchisy[0];
	highest[2] = acchisz[0];
	highest[3] = rolhisx[0];
	highest[4] = rolhisy[0];
	highest[5] = rolhisz[0];

	lowest[0] = acchisx[0];
	lowest[1] = acchisy[0];
	lowest[2] = acchisz[0];
	lowest[3] = rolhisx[0];
	lowest[4] = rolhisy[0];
	lowest[5] = rolhisz[0];
	
	for(i = 1; i < 50; i ++){
		if(acchisx[i] < lowest[0]){
			lowest[0] = acchisx[i];
		}
		else if(acchisx[i] > highest[0]){
			highest[0] = acchisx[i];
		}
		if(acchisy[i] < lowest[1]){
			lowest[1] = acchisy[i];
		}
		else if(acchisy[i] > highest[1]){
			highest[1] = acchisy[i];
		}
		if(acchisz[i] < lowest[2]){
			lowest[2] = acchisz[i];
		}
		else if(acchisz[i] > highest[2]){
			highest[2] = acchisz[i];
		}	

		if(rolhisx[i] < lowest[3]){
			lowest[3] = rolhisx[i];
		}
		else if(rolhisx[i] > highest[3]){
			highest[3] = rolhisx[i];
		}
		if(rolhisy[i] < lowest[4]){
			lowest[4] = rolhisy[i];
		}
		else if(rolhisy[i] > highest[4]){
			highest[4] = rolhisy[i];
		}
		if(rolhisz[i] < lowest[5]){
			lowest[5] = rolhisz[i];
		}
		else if(rolhisz[i] > highest[5]){
			highest[5] = rolhisz[i];
		}
		acceltotal[0] += acchisx[i];
		acceltotal[1] += acchisy[i];
		acceltotal[2] += acchisz[i];
		rolltotal[0] += rolhisx[i];
		rolltotal[1] += rolhisy[i];
		rolltotal[2] += rolhisz[i];
	}
	
	for(i = 0; i < 6; i ++){
		if((lowest[i] + 20) < highest[i]){
			return 0;
		}
	}
	
	for(i = 0; i < 3; i ++){
		accelnorm[i] = acceltotal[i] / 50;
		rollnorm[i] = rolltotal[i] / 50;
	}
	return 1;
}

/*TWI Master Initiate*/
void twiInitiate(TWI_Master_t *title,TWI_t *interface){
	uint8_t accelsetupbuffer1[3] = {0x2C, 0b00001000, 0x08};
	uint8_t accelsetupbuffer2[3] = {0x31, 0x00};
	uint8_t rollsetupbuffer1[4] = {0x15, 19, 0b00011100, 0x11};
	uint8_t rollsetupbuffer2[] = {0x3E, 0b00000001};

	TWI_MasterInit(title, interface, TWI_MASTER_INTLVL_HI_gc, TWI_BAUDSETTING);

	while(title->status != TWIM_STATUS_READY);
	PORTF.OUT = 1;
	TWI_MasterWriteRead(title, ACCEL, accelsetupbuffer1, 3, 0);
	while(title->status != TWIM_STATUS_READY);
	PORTF.OUT = 2;
	TWI_MasterWriteRead(title, ACCEL, accelsetupbuffer2, 2, 0);
	while(title->status != TWIM_STATUS_READY);
	PORTF.OUT = 3;
	TWI_MasterWriteRead(title, ROLL, rollsetupbuffer1, 4, 0);
	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, ROLL, rollsetupbuffer2, 2, 0);
	while(title->status != TWIM_STATUS_READY);
	interface->MASTER.CTRLB |= 0x0C;
}


/*USART Initiate*/
void uartInitiate(USART_data_t * title,USART_t * interface){
	USART_InterruptDriver_Initialize(title, interface, USART_DREINTLVL_LO_gc);
	USART_Format_Set(title->usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(title->usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(interface, 12 , 0);
	USART_Rx_Enable(title->usart);
	USART_Tx_Enable(title->usart);
}

/*Update Offset*/
char updateoffset(TWI_Master_t * imu,
		int * accelnorm,
		char * rollnorm,
		char * rolhisx,
		char * rolhisy,
		char * rolhisz,
		int * acchisx,
		int * acchisy,
		int * acchisz,
		int * accelcash,
		char * rollcash, 
		char * readyset, 
		uint8_t * rollstartbyte, 
		uint8_t * accelstartbyte){
	int i;
	int j;

	if(!*readyset){
		for(i = 0; i < 50; i ++){
			for(j = 0; j < 3; j ++){
				rollcash[j] = 0;
				accelcash[j] = 0;
			}
			while(!(TCC0.INTFLAGS & 0x01));
			TCC0.INTFLAGS |= 0x01;
			getroll(rollcash, imu, rollstartbyte);
			getaccel(accelcash, imu, accelstartbyte);

			acchisx[i] = accelcash[0];
			acchisy[i] = accelcash[1];
			acchisz[i] = accelcash[2];
			rolhisx[i] = rollcash[0];
			rolhisy[i] = rollcash[1];
			rolhisz[i] = rollcash[2];
		}
		//*readyset = 1; for real time offsetting
	}

	else{
		for(i = 49; i > 0; i --){
			acchisx[i] = acchisx[i - 1];
			acchisy[i] = acchisy[i - 1];
			acchisz[i] = acchisz[i - 1];
			rolhisx[i] = rolhisx[i - 1];
			rolhisy[i] = rolhisy[i - 1];
			rolhisz[i] = rolhisz[i - 1];
		}
		getroll(rollcash, imu, rollstartbyte);
		getaccel(accelcash, imu, accelstartbyte);
		acchisx[0] = accelcash[0];
		acchisy[0] = accelcash[1];
		acchisz[0] = accelcash[2];
		rolhisx[0] = rollcash[0];
		rolhisy[0] = rollcash[1];
		rolhisz[0] = rollcash[2];
	}
	return getoffset(acchisx, acchisy, acchisz, rolhisx, rolhisy, rolhisz, accelnorm, rollnorm);
}

int absval(int value){
	if(value < 0){
		return value * -1;
	}
	else{
		return value;
	}
}
