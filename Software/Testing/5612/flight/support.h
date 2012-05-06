

/**
@author Daniel Sidlauskas Miller

*/

#define CPU_SPEED 2000000

#define DEGREE 40

#define AWEIGHT 1
#define GWEIGHT 9

#define ROTJOYSENS 1
#define ZJOYSENS -2
#define TILTJOYSENS 3/4
#define MOTORREG 3000

//Rate which gyro will be read at
#define RATE 200

#define BAUDRATE 200000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

#define GYROINT 0
#define ACCELINT 0

//TC overflow flag stuff
#define TC_GetOverflowFlag( _tc ) ( (_tc)->INTFLAGS & TC0_OVFIF_bm )
#define TC_ClearOverflowFlag( _tc ) ( (_tc)->INTFLAGS = TC0_OVFIF_bm )


#define BIG_ENDIAN      0

#define LITTLE_ENDIAN   1


void sprintBinary(char * buffer, int * value);

void sendchar( USART_data_t * uart, char buffer);

void sendstring( USART_data_t * uart, char *buffer);

void CCPWrite( volatile uint8_t * address, uint8_t value );

void twiInitiate(TWI_Master_t * title,TWI_t * interface);

void uartInitiate(USART_data_t * title,USART_t * interface);

void motorSpeed(int * pry,
		int * integration, 
		int * gyroint, 
		int * joystick, 
		int * motorSpeeds,
		int * pidValues,
		int * pidValuesDen);
