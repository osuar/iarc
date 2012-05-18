#include "../../drivers/twi_master_driver.h"
#include "../../drivers/avr_compiler.h"
#include "adxl345.h"



void adxl345Init(TWI_Master_t *title){
	uint8_t accelsetupbuffer1[3] = {0x2C, 0b00001010, 0x08};
	uint8_t accelsetupbuffer2[3] = {0x31, 0x00};

	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, ACCEL, accelsetupbuffer1, 3, 0);
	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, ACCEL, accelsetupbuffer2, 2, 0);
	while(title->status != TWIM_STATUS_READY);
}

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
			accelcache[i] = (accelcache[i] * 2);
	}
}





