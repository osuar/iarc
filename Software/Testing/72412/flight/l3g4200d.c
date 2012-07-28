#include "../../drivers/twi_master_driver.h"
#include "../../drivers/avr_compiler.h"
#include "l3g4200d.h"


void l3g4200d_init(TWI_Master_t *title){
	unsigned char gyro_setup_buffer_1[] = {0x20, 0b01100111};

	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, GYRO, gyro_setup_buffer_1, 2, 0);
	while(title->status != TWIM_STATUS_READY);
}

void getgyro(int * gyrocache, TWI_Master_t *imu){
	unsigned char gyro_check = 0x27;
	unsigned char gyro_read = 0x28;

	int i;

	do{
		while(title->status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(title, GYRO, gyro_check, 1, 1);
		while(title->status != TWIM_STATUS_READY);
	}while(!(imu->readData[0] & 0b00001000));

	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, GYRO, gyro_read, 1, 6);
	while(title->status != TWIM_STATUS_READY);

	
	for(i = 0; i < 3; i++){
		gyrocache[i] = (imu->readData[2*i] & 0xff) | (imu->readData[(2*i) + 1] << 8);
	}
}
	

