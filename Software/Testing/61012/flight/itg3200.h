#define GYRO (0xD0 >> 1)

void itg3200Init(TWI_Master_t *title, int rate);

void getgyro(int *gyrocache, TWI_Master_t *imu, uint8_t *gyrostartbyte);
