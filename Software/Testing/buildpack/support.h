
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
#define SERVOLINI 3500
#define SERVORINI 3170
#define MOTORINI 2000
#define MOTORRNORM 2660
#define MOTORLNORM 2640

//MIN/MAX motor
#define MAXMOTOR 3500
#define MINMOTOR 2550
#define MAXSERVO 3590
#define MINSERVO 2790

//Change constants
#define SERVORAX 4
#define SERVOLAX 4
#define MOTORRAX 0
#define MOTORLAX 0
#define SERVORRX 0
#define SERVOLRX 0
#define MOTORRRX 1
#define MOTORLRX -1

#define SERVORAY 0
#define SERVOLAY 0
#define MOTORRAY -1
#define MOTORLAY 1
#define SERVORRY 0
#define SERVOLRY 0
#define MOTORRRY -1
#define MOTORLRY 1


#define SERVORAZ 0
#define SERVOLAZ 0
#define MOTORRAZ -1 
#define MOTORLAZ -1
#define SERVORRZ -6
#define SERVOLRZ 6
#define MOTORRRZ 0
#define MOTORLRZ 0


//TC overflow flag stuff
#define TC_GetOverflowFlag( _tc ) ( (_tc)->INTFLAGS & TC0_OVFIF_bm )
#define TC_ClearOverflowFlag( _tc ) ( (_tc)->INTFLAGS = TC0_OVFIF_bm )


#define BIG_ENDIAN      0

#define LITTLE_ENDIAN   1


void CCPWrite( volatile uint8_t * address, uint8_t value );

void ValueFunk(int accelx, int accely, int accelz, int rollx, int rolly, int rollz, short int *servol, short int *servor, short int *motorl, short int *motorr);

unsigned char exponent(char* bytes);

char output(char* bytes, unsigned char min);

unsigned char outputlong(char* bytes, unsigned char min);
