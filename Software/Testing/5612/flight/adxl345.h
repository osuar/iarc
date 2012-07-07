#define ACCEL (0xA6 >> 1)

void adxl345Init(TWI_Master_t *title);

void getaccel(int *accelcache, TWI_Master_t *imu, uint8_t *accelstartbyte);
