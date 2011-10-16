
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

//IMU static values
#define AXNORM 0
#define AZNORM 266

//initial motor constants
#define SERVOINI 3000
#define MOTORINI 2000
#define MOTORRNORM 3250
#define MOTORLNORM 3250

//Change constants
#define SERVORAX 2
#define SERVOLAX -2
#define MOTORRAX 0
#define MOTORLAX 0

#define SERVORAY 0
#define SERVOLAY 0
#define MOTORRAY 1
#define MOTORLAY -1

#define SERVORAZ 0
#define SERVOLAZ 0
#define MOTORRAZ 1 
#define MOTORLAZ 1

//TC overflow flag stuff
#define TC_GetOverflowFlag( _tc ) ( (_tc)->INTFLAGS & TC0_OVFIF_bm )
#define TC_ClearOverflowFlag( _tc ) ( (_tc)->INTFLAGS = TC0_OVFIF_bm )

void ValueFunk(int accelx, int accely, int accelz, int rollx, int rolly, int rollz, volatile int *servol, volatile int *servor, volatile int *motorl, volatile int *motorr);


