
/**
@author Daniel Sidlauskas Miller with advice from Gary Miller

This header file includes contants and prototype functions for flightcontrol.c
Source for functions prototyped here found in aux.c
*/

#define CPU_SPEED 2000000

//Rate which gyro will be read at
#define RATE 200

//imu constants
#define ACCEL (0xA6 >> 1)
#define BAUDRATE 200000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)
#define GYRO (0xD0 >> 1)

#define DAMPENGYRO 1
#define INTEGRATEGYRO 0
#define DAMPENACCEL 1
#define INTEGRATEACCEL 0

//Ir position and acceleration constants, Upper values are for closer IR senser readings
#define HORIZVEL 0
#define VERTVEL 0
#define HORIZUPPER 27
#define HORIZLOWER 25
#define VERTUPPER 51
#define VERTLOWER 45
#define VERTAU 0
#define HORIZAF 0

//IMU static values
#define AXN 0
#define AYN 0
#define AZN 0
#define RXN 0
#define RYN 0
#define RZN 0

//initial motor constants
#define MOTOR2NORM 2000
#define MOTOR4NORM 2000
#define MOTORINI 2000
#define MOTOR3NORM 2000
#define MOTOR1NORM 2000 
//MIN/MAX motor
#define MAXMOTOR 4000
#define MINMOTOR 2000
#define MAXSERVO 3400
#define MINSERVO 2500

//Change constants
#define MOTOR4AX -12 //-8
#define MOTOR2AX 12 //8
#define MOTOR3AX 0
#define MOTOR1AX 0
#define MOTOR4RX 0
#define MOTOR2RX -0
#define MOTOR3RX 8 //12
#define MOTOR1RX 8 //-12

#define MOTOR4AY 0
#define MOTOR2AY 0
#define MOTOR3AY 12 //8
#define MOTOR1AY -12 //-8
#define MOTOR4RY 8 //12
#define MOTOR2RY -8 //-12
#define MOTOR3RY 0
#define MOTOR1RY -0


#define MOTOR4AZ -0
#define MOTOR2AZ 0
#define MOTOR3AZ 0
#define MOTOR1AZ 0
#define MOTOR4RZ 0
#define MOTOR2RZ 0
#define MOTOR3RZ 0
#define MOTOR1RZ -0

#define GYROINT 0
#define ACCELINT 0

//TC overflow flag stuff
#define TC_GetOverflowFlag( _tc ) ( (_tc)->INTFLAGS & TC0_OVFIF_bm )
#define TC_ClearOverflowFlag( _tc ) ( (_tc)->INTFLAGS = TC0_OVFIF_bm )


#define BIG_ENDIAN      0

#define LITTLE_ENDIAN   1

void getgyro(int *gyrocache, TWI_Master_t *imu, uint8_t *gyrostartbyte);

void sprintBinary(char * buffer, int * value);

void getaccel(int *accelcache, TWI_Master_t *imu, uint8_t *accelstartbyte);

void sendchar( USART_data_t * uart, char buffer);

void sendstring( USART_data_t * uart, char *buffer);

void CCPWrite( volatile uint8_t * address, uint8_t value );

void ValueFunk(int accelx, int accely, int accelz, int gyrox, int gyroy, int gyroz, short int *motor2, short int *motor4, short int *motor1, short int *motor3);

unsigned char exponent(char* bytes);

char output(char* bytes, unsigned char min);

unsigned char outputlong(char* bytes, unsigned char min);

char getoffset(int *acchisx,int * acchisy,int * acchisz,char * rolhisx,char * rolhisy,char * rolhisz,int * accelnorm,char * gyronorm);

void twiInitiate(TWI_Master_t * title,TWI_t * interface);

void uartInitiate(USART_data_t * title,USART_t * interface);

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
		uint8_t * accelstartbyte);
	
void updateDCM();

int absval(int value);
