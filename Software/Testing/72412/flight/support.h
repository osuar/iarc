

/**
@author Daniel Sidlauskas Miller

*/

#define FORWARD_SERVO_DEFAULT 900
#define RIGHT_SERVO_DEFAULT 600
#define LEFT_SERVO_DEFAULT 1000

#define CPU_SPEED 32000000

#define DEGREE 40
#define PI 2450

#define AWEIGHT 2
#define GWEIGHT 8

#define ROTJOYSENSDOWN 2
#define ROTJOYSENSUP 3/2
#define ZJOYSENS 3/2
#define THROTTLEMAIN 6
#define PRJOYDIF13 3/2
#define PRJOYDIF24 3/2
#define TILTJOYSENS24 3/4
#define TILTJOYSENS13 3/4
#define THROTTLEJOYDIF 5
#define YAWJOYDIF 3
#define MOTORREG 2900

#define YAW_INT_MAX 2000

//Rate which gyro will be read at
#define RATE 200

#define BAUDRATE 200000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

#define GYROINT 0
#define ACCELINT 1

//TC overflow flag stuff
#define TC_GetOverflowFlag( _tc ) ( (_tc)->INTFLAGS & TC0_OVFIF_bm )
#define TC_ClearOverflowFlag( _tc ) ( (_tc)->INTFLAGS = TC0_OVFIF_bm )


#define BIG_ENDIAN      0

#define LITTLE_ENDIAN   1


int arctan2(int opp, int adj);

void sprintBinary(char * buffer, int * value);

void sendchar( USART_data_t * uart, char buffer);

void sendstring( USART_data_t * uart, char *buffer);

void sendpacket(USART_data_t * uart, char *buffer);

void CCPWrite( volatile uint8_t * address, uint8_t value );

void twiInitiate(TWI_Master_t * title,TWI_t * interface);

void uartInitiate(USART_data_t * title,USART_t * interface,unsigned long int baudrate);

void motorSpeed(int * pry,
		int * integration, 
		int * gyroint, 
		int * joystick, 
		int * motorSpeeds,
		int * pidValues13,
		int * pidValues24,
		int * pidValuesDen13,
		int * pidValuesDen24);

void yawCorrect(int * motorSpeeds, int * gyroint,int * integration, int * roterr, char * pidRotUp, char * pidRotDenUp, char * pidRotDown, char * pidRotDenDown);

