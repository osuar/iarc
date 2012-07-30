#define GYRO 0b11010000

void l3g4200d_init(TWI_Master_t *title);

void getgyro(int * gyrocache, TWI_Master_t *imu);



