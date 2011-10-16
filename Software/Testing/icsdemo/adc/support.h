
/**
@author Daniel Sidlauskas Miller with advice from Gary Miller

This header file includes contants and prototype functions for flightcontrol.c
Source for functions prototyped here found in aux.c
*/

#define CPU_SPEED 2000000

//imu constants
#define ACCEL (0xA6 >> 1)
#define BAUDRATE 100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)
#define ROLL (0xD0 >> 1)

#define DAMPENROLL 1
#define INTEGRATEROLL 0
#define DAMPENACCEL 1
#define INTEGRATEACCEL 0

//IMU static values
#define AXN 0
#define AYN 0
#define AZN 0
#define RXN 0
#define RYN 0
#define RZN 0

//initial motor constants
#define SERVOLINI 3015
#define SERVORINI 2865
#define MOTORINI 2000
#define MOTORRNORM 3785
#define MOTORLNORM 3845

//MIN/MAX motor
#define MAXMOTOR 4000
#define MINMOTOR 2600
#define MAXSERVO 3400
#define MINSERVO 2500

//Change constants
#define SERVORAX 0
#define SERVOLAX 0
#define MOTORRAX 8
#define MOTORLAX -8
#define SERVORRX 0
#define SERVOLRX 0
#define MOTORRRX 0
#define MOTORLRX 0

#define SERVORAY 0
#define SERVOLAY 0
#define MOTORRAY 0
#define MOTORLAY 0
#define SERVORRY 0
#define SERVOLRY 0
#define MOTORRRY 0
#define MOTORLRY -0


#define SERVORAZ -0
#define SERVOLAZ 0
#define MOTORRAZ 0 
#define MOTORLAZ 0
#define SERVORRZ 0
#define SERVOLRZ 0
#define MOTORRRZ 8
#define MOTORLRZ -8


//TC overflow flag stuff
#define TC_GetOverflowFlag( _tc ) ( (_tc)->INTFLAGS & TC0_OVFIF_bm )
#define TC_ClearOverflowFlag( _tc ) ( (_tc)->INTFLAGS = TC0_OVFIF_bm )


#define BIG_ENDIAN      0

#define LITTLE_ENDIAN   1

void getroll(char *rollcash, TWI_Master_t *imu, uint8_t *rollstartbyte);

void getaccel(int *accelcash, TWI_Master_t *imu, uint8_t *accelstartbyte);

void sendchar( USART_data_t * uart, char buffer);

void sendstring( USART_data_t * uart, char *buffer);

void CCPWrite( volatile uint8_t * address, uint8_t value );

void ValueFunk(int accelx, int accely, int accelz, int rollx, int rolly, int rollz, short int *servol, short int *servor, short int *motorl, short int *motorr);

unsigned char exponent(char* bytes);

char output(char* bytes, unsigned char min);

unsigned char outputlong(char* bytes, unsigned char min);

char getoffset(int *acchisx,int * acchisy,int * acchisz,char * rolhisx,char * rolhisy,char * rolhisz,int * accelnorm,char * rollnorm);

void twiInitiate(TWI_Master_t * title,TWI_t * interface);

void uartInitiate(USART_data_t * title,USART_t * interface);

char updateoffset(TWI_Master_t * imu,
		int * accelnorm,
		char * rollnorm,
		char * rolhisx,
		char * rolhisy,
		char * rolhisz,
		int * acchisx,
		int * acchisy,
		int * acchisz,
		int * accelcash,
		char * rollcash, 
		char * readyset, 
		uint8_t * rollstartbyte, 
		uint8_t * accelstartbyte);
	
int absval(int value);
