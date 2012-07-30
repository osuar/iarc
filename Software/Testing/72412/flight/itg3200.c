#include "../../drivers/twi_master_driver.h"
#include "../../drivers/avr_compiler.h"
#include "itg3200.h"

void itg3200Init(TWI_Master_t *title, int rate){
	uint8_t gyrosetupbuffer1[4] = {0x15, (1000/rate) - 1, 0b00011000, 0x11};
	uint8_t gyrosetupbuffer2[] = {0x3E, 0b00000010};

	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, GYRO, gyrosetupbuffer1, 4, 0);
	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, GYRO, gyrosetupbuffer2, 2, 0);
	while(title->status != TWIM_STATUS_READY);
}

void getgyro(int *gyrocache, TWI_Master_t *imu, uint8_t *gyrostartbyte){
	int i;
	uint8_t buffer[6];
//	do{
		while(imu->status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(imu, GYRO, gyrostartbyte, 1, 10);
		while(imu->result == TWIM_STATUS_READY);
//	}while(!(imu->readData[0] & 0x01));
	for(i = 0; i <= 5; i ++){
		buffer[i] = imu->readData[i + 3];
	}
	for(i = 0; i < 5; i += 2){
		gyrocache[i/2] = ((buffer[i] << 8) | buffer[i + 1]) >> 3;
		if(abs(gyrocache[i/2] ) < 2){
	//		gyrocache[i/2] = 0;
		}
	}
}
