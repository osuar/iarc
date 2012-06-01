#define MAG 0b00011110

void lsm303dlhInit(TWI_Master_t *title);

void getmag(int * magcache, TWI_Master_t *title);
