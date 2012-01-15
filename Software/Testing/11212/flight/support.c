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




void ValueFunk(int accelx, int accely, int accelz, int gyrox, int gyroy, int gyroz, short int *motor2, short int *motor4, short int *motor1, short int *motor3){
	
	accely += absval(accelx);
	accely += absval(accelz);

	*motor2 = MOTOR2NORM;
	*motor2 += MOTOR2AX * accelx;
	*motor2 += MOTOR2AY * accely;
	*motor2 += MOTOR2AZ * accelz;
	*motor2 += MOTOR2RX * gyrox * absval(gyrox);
	*motor2 += MOTOR2RY * gyroy * absval(gyroy) / 8;
	*motor2 += MOTOR2RZ * gyroz;
	if(*motor2 >= MAXMOTOR){
		*motor2 = MAXMOTOR;
	}
	if(*motor2 <= MINMOTOR){
		*motor2 = MINMOTOR;
	}

	*motor4 = MOTOR4NORM;
	*motor4 += MOTOR4AX * accelx;
	*motor4 += MOTOR4AY * accely;
	*motor4 += MOTOR4AZ * accelz;
	*motor4 += MOTOR4RX * gyrox * absval(gyrox);
	*motor4 += MOTOR4RY * gyroy * absval(gyroy) / 8;
	*motor4 += MOTOR4RZ * gyroz;
	if(*motor4 >= MAXMOTOR){
		*motor4 = MAXMOTOR;
	}
	if(*motor4 <= MINMOTOR){
		*motor4 = MINMOTOR;
	}
	

	*motor1 = MOTOR1NORM;

	*motor1 += MOTOR1AX * accelx;
	*motor1 += MOTOR1AY * accely;
	*motor1 += MOTOR1AZ * accelz;
	*motor1 += MOTOR1RX * gyrox;
	*motor1 += MOTOR1RY * gyroy;
	*motor1 += MOTOR1RZ * gyroz;


	if(*motor1 >= MAXMOTOR){
		*motor1 = MAXMOTOR;
	}
	if(*motor1 <= MINMOTOR){
		*motor1 = MINMOTOR;
	}


	*motor3 = MOTOR3NORM;
	*motor3 += MOTOR3AX * accelx;
	*motor3 += MOTOR3AY * accely;
	*motor3 += MOTOR3AZ * accelz;
	*motor3 += MOTOR3RX * gyrox;
	*motor3 += MOTOR3RY* gyroy;
	*motor3 += MOTOR3RY * gyroy;
	*motor3 += MOTOR3RZ * gyroz;
	if(*motor3 >= MAXMOTOR){
		*motor3 = MAXMOTOR;
	}
	if(*motor3 <= MINMOTOR){
		*motor3 = MINMOTOR;
	}

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
			

/* get gyro data, should do test to figure out degree/sec magnitude*/ 
void getgyro(int *gyrocache, TWI_Master_t *imu, uint8_t *gyrostartbyte){
	int i;
	char buffer[6];
	do{
		while(imu->status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(imu, GYRO, gyrostartbyte, 1, 10);
		while(imu->result == TWIM_RESULT_UNKNOWN);
	}while(!(imu->readData[0] & 0x01));
	for(i = 0; i < 5; i ++){
		buffer[i] = imu->readData[i + 3];
	}
	for(i = 0; i < 5; i += 2){
	//	gyrocache[i/2] = (buffer[i] << 8) | buffer[i + 1];
//		gyrocache[i/2] = buffer[i] << 8;
//		gyrocache[i/2] |= (buffer[i + 1] & 0x7F);
		gyrocache[i/2] = buffer[i];
	}
}

/* get accelerometer data, 1 g ~= 62 LSB*/
void getaccel(int *accelcache, TWI_Master_t *imu, uint8_t *accelstartbyte){
	int i;
	do{
		while(imu->status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(imu, ACCEL, accelstartbyte, 1, 10);
		while(imu->result == TWIM_RESULT_UNKNOWN);
	}while(!(imu->readData[0] & 0x80));

	for(i = 0; i < 5; i += 2){
		if(imu->readData[i + 3] & 0x80){
			accelcache[i/2] -= 256 * (~imu->readData[i + 3] + 1);
			accelcache[i/2] -= (~imu->readData[i + 2] + 1);
		}
		else{
			accelcache[i/2] += 256 * imu->readData[i + 3];
			accelcache[i/2] += (imu->readData[i + 2]);
		}
	}
	for(i = 0; i < 3; i ++){
			accelcache[i] = (accelcache[i] >> 2) * 3;
	}
}

char getoffset(int *acchisx,int * acchisy,int * acchisz,char * rolhisx,char * rolhisy,char * rolhisz,int * accelnorm,char * gyronorm){
	int lowest[6];
	int highest[6];
	int gyrototal[3] = {0,0,0};
	long long int acceltotal[3] = {0,0,0};
	int i;

/*
	for(i = 0; i < 3; i ++){
		accelnorm[i] = 0;
		gyronorm[i] = 0;
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
		gyrototal[0] += rolhisx[i];
		gyrototal[1] += rolhisy[i];
		gyrototal[2] += rolhisz[i];
	}
	
	for(i = 0; i < 6; i ++){
		if((lowest[i] + 20) < highest[i]){
			return 0;
		}
	}
	
	for(i = 0; i < 3; i ++){
		if(gyrototal[i] < -45){
			gyrototal[i] -= 5;
		}
		else if(gyrototal[i] > 0){
			gyrototal[i] += 0;
		}
		accelnorm[i] = acceltotal[i] / 50;
		gyronorm[i] = gyrototal[i] / 50;
	}
	return 1;
}

/*TWI Master Initiate*/
void twiInitiate(TWI_Master_t *title,TWI_t *interface){
	uint8_t accelsetupbuffer1[3] = {0x2C, 0b00001001, 0x08};
	uint8_t accelsetupbuffer2[3] = {0x31, 0x00};
	uint8_t gyrosetupbuffer1[4] = {0x15, (1000/RATE) - 1, 0b00011000, 0x11};
	uint8_t gyrosetupbuffer2[] = {0x3E, 0b00000001};

	TWI_MasterInit(title, interface, TWI_MASTER_INTLVL_HI_gc, TWI_BAUDSETTING);

	//For some reason, gryo must be initiated first
	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, GYRO, gyrosetupbuffer1, 4, 0);
	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, GYRO, gyrosetupbuffer2, 2, 0);
	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, ACCEL, accelsetupbuffer1, 3, 0);
	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, ACCEL, accelsetupbuffer2, 2, 0);
	while(title->status != TWIM_STATUS_READY);

	interface->MASTER.CTRLB |= 0x0C;
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

/*Update Offset*/
char updateoffset(TWI_Master_t * imu,
		int * accelnorm,
		char * gyronorm,
		char * rolhisx,
		char * rolhisy,
		char * rolhisz,
		int * acchisx,
		int * acchisy,
		int * acchisz,
		int * accelcache,
		int * gyrocache, 
		char * readyset, 
		uint8_t * gyrostartbyte, 
		uint8_t * accelstartbyte){
	int i;
	int j;

	if(!*readyset){
		for(i = 0; i < 50; i ++){
			for(j = 0; j < 3; j ++){
				gyrocache[j] = 0;
				accelcache[j] = 0;
			}
			while(!(TCC0.INTFLAGS & 0x01));
			TCC0.INTFLAGS |= 0x01;
			getgyro(gyrocache, imu, gyrostartbyte);
			getaccel(accelcache, imu, accelstartbyte);

			acchisx[i] = accelcache[0];
			acchisy[i] = accelcache[1];
			acchisz[i] = accelcache[2];
			rolhisx[i] = gyrocache[0];
			rolhisy[i] = gyrocache[1];
			rolhisz[i] = gyrocache[2];
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
		getgyro(gyrocache, imu, gyrostartbyte);
		getaccel(accelcache, imu, accelstartbyte);
		acchisx[0] = accelcache[0];
		acchisy[0] = accelcache[1];
		acchisz[0] = accelcache[2];
		rolhisx[0] = gyrocache[0];
		rolhisy[0] = gyrocache[1];
		rolhisz[0] = gyrocache[2];
	}
	return getoffset(acchisx, acchisy, acchisz, rolhisx, rolhisy, rolhisz, accelnorm, gyronorm);
}

int absval(int value){
	if(value < 0){
		return value * -1;
	}
	else{
		return value;
	}
}
