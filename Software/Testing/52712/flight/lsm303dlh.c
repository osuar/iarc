#include "../../drivers/twi_master_driver.h"
#include "../../drivers/avr_compiler.h"
#include "lsm303dlh.h"

void lsm303dlhInit(TWI_Master_t *title){
	uint8_t magsetupbuffer1[3] = {0x00, 0b00010100, 0b00100000};

	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, MAG, magsetupbuffer1, 3, 0);
	while(title->status != TWIM_STATUS_READY);
}
	

void getmag(TWI_Master_t *imu, int * magcache){
	char magstartbyte = 0x03;
	int i;
		do{
		while(imu->status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(imu, MAG, magstartbyte, 1, 7);
		while(imu->result == TWIM_RESULT_UNKNOWN);
	}while(!(imu->readData[6] & 0x01));

	for(i = 0; i < 3; i ++){
		magcache[i] = (imu->readData[2*i] << 8) | (imu->readData[2*i + 1]);
	}
}

	



