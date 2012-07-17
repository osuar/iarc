#include "../../drivers/twi_master_driver.h"
#include "../../drivers/avr_compiler.h"
#include "lsm303dlh.h"

void lsm303dlhInit(TWI_Master_t *title){
	uint8_t magsetupbuffer1[3] = {0, 0b00010100, 0b11100000};
	uint8_t magsetupbuffer2[3] = {2, 0b00000000};


	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, MAG, magsetupbuffer1, 4, 0);
	while(title->status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(title, MAG, magsetupbuffer2, 2, 0);
	while(title->status != TWIM_STATUS_READY);

}
	

void getmag(int * magcache, TWI_Master_t *imu){
	unsigned char magstartbyte = 0x03;
	int magprev;
	int i;
		//do{
		while(imu->status != TWIM_STATUS_READY);
		TWI_MasterWriteRead(imu, MAG, &magstartbyte, 1, 6);
		while(imu->result == TWIM_RESULT_UNKNOWN);
	//}while(!(imu->readData[6] & 0x01));

	for(i = 0; i < 3; i ++){
		magprev = magcache[i];
		magcache[i] = ((imu->readData[2*i] << 8) | (imu->readData[2*i + 1]));
		magcache[i] = magcache[i] >> 0;
		//magcache[i] = (magcache[i] + magprev)/2;
		//magcache[i] = imu->readData[i];
		if(abs(magcache[i]) > 200){
			magcache[i] = 0;
		}
	}
	magcache[0] += 30;
	magcache[1] += 10;
	magcache[1] = (magcache[1] * 10)/9;
	
}

	



