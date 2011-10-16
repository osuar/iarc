#include <stdio.h>
#include "avr_compiler.h"
#include "usart_driver.h"
#include "twi_master_driver.h"
#include "support.h"

#define AVR_ENTER_CRITICAL_REGION( ) uint8_t volatile saved_sreg = SREG; \
                                     cli();

#define AVR_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;

void CCPWrite( volatile uint8_t * address, uint8_t value )
{
#if defined __GNUC__
AVR_ENTER_CRITICAL_REGION( );
volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
RAMPZ = 0;
#endif
asm volatile(
"movw r30,  %0"      "\n\t"
"ldi  r16,  %2"      "\n\t"
"out   %3, r16"      "\n\t"
"st     Z,  %1"       "\n\t"
:
: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
: "r16", "r30", "r31"
);

AVR_LEAVE_CRITICAL_REGION( );
#endif
}



USART_data_t IMU;
USART_data_t xbee;


volatile int readdata = 0;

int main(void){
	int i;
	int j;
	int bytetobuffer;
	char xbeebuffer[100];
	char receive[100];
	char input;
	int count = 0;

	int rollcash[3] = {0,0,0};
	int accelcash[3] = {0,0,0};

	short int motorr = 0;
	short int motorl = 0;
	short int servor = 0;
	short int servol = 0;

	/**Move cmd vars*/
	short int rise = 0;
	short int rotate = 0;
	short int forward = 0;
	short int tilt = 0;

	char imuread = 0xD2;

	enum states {runningread, stopped} state = stopped;

	/**Setup directions for serial interfaces*/
	PORTC.DIR = 0b00001000;
	PORTE.DIR |= 0b00001000;
	PORTF.DIR |= 0x03;
	PORTD.DIR = 0x0F;

	//Pulse width modulation setup for servos, port D
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 40000;

	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 10000;

	/**Setup interrupts*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();


	/**Setup IMU*/
	USART_InterruptDriver_Initialize(&IMU, &USARTE0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(IMU.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(IMU.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 19 , -4);
	USARTE0.CTRLB |= 0x04;
	USART_Rx_Enable(IMU.usart);
	USART_Tx_Enable(IMU.usart);


	/**Setup Xbee*/
	USART_InterruptDriver_Initialize(&xbee, &USARTC0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTC0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);
	

	while(1){



	if(USART_RXBufferData_Available(&xbee)){
		PORTF.OUT |= 0x01;
		input = USART_RXBuffer_GetByte(&xbee);
		if(input == 'r'){
			state = runningread;
			sprintf(xbeebuffer, "read\n\r");
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
		}
		else if(input == 's'){
			state = stopped; 
			sprintf(xbeebuffer, "stop\n\r");
			for(i = 0; i < 3; i ++){
				accelcash[i] = 0;
				rollcash[i] = 0;
			}
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
			TCD0.CCA = 2000;
			TCD0.CCB = SERVOLINI;
			TCD0.CCC = 2000;
			TCD0.CCD = SERVORINI;
			sprintf(xbeebuffer, "rise %4d tilt %4d rot %4d for %4d \n\r", rise, tilt, rotate, forward);
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
		}
		else if(input == 'q'){
			CCPWrite( &RST.CTRL, 1 );
			while(1);
		}
		else if(input == 'u'){
			rise += 25;
		}
		else if(input == 'd'){
			rise -= 25;
		}
		else if(input == 'c'){
			rotate += 10;
		}
		else if(input == 'x'){
			rotate -= 10;
		}
		else if(input == 'a'){
			tilt += 10;
		}
		else if(input == 'e'){
			tilt -= 10;
		}
		else if(input == 't'){
			forward += 10;
		}
		else if(input == 'b'){
			forward -= 10;
		}
	}

	switch(state){

	case stopped:
	PORTF.OUT |= 0x02;
	TCD0.CCA = 2000;
	TCD0.CCB = SERVOLINI;
	TCD0.CCC = 2000;
	TCD0.CCD = SERVORINI;
	break;	

	case runningread:

		PORTF.OUT = 0;
		if(TCC0.INTFLAGS & 0x01){
			TCC0.INTFLAGS = 0x01;
			USART_TXBuffer_PutByte(&IMU, imuread);
			while(!USART_RXBufferData_Available(&IMU));
			PORTF.OUT ^= 0x02;
			receive[42] = USART_RXBuffer_GetByte(&IMU);		
			for(j = 1; j < 43; j ++){
				while(!USART_RXBufferData_Available(&IMU));
				receive[42 - j] = USART_RXBuffer_GetByte(&IMU);
			}
			PORTF.OUT |= 0x01;
			for(j = 0; j < 3; j ++){
				PORTF.OUT |= 0x02;
				accelcash[j] += output(&receive[38 - (4 * j )], 0b01111000);
				rollcash[j] += output(&receive[26 - (4 * j)], 0b01111000);

			}
			
			PORTF.OUT |= 0x02;
			for(j = 0; xbeebuffer[j] != 0 ; j ++){
				do{
				bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[j]);
				}while(!bytetobuffer);
			}

			count ++;
			rollcash[0] -= RXN;
			rollcash[1] -= RYN;
			rollcash[2] -= RZN;
			accelcash[0] -= AXN;
			accelcash[1] -= AYN;
			accelcash[2] -= AZN;


		}

		if(count >= 4){
			count = 0;
			PORTF.OUT = 3;
			for(i = 0; i < 3; i ++){
				accelcash[i] /= DAMPENACCEL;
				rollcash[i] /= DAMPENROLL;
			}


		//motor updates
	

			ValueFunk(accelcash[0],accelcash[1],accelcash[2],rollcash[0],rollcash[1],rollcash[2],&servol,&servor,&motorl,&motorr);
			while(TCD0.CNT < 4000);

			TCD0.CCA = motorl + rise - tilt;
			TCD0.CCB = servol + rotate + forward;
			TCD0.CCC = motorr + rise + tilt;
			TCD0.CCD = servor - rotate + forward;

			sprintf(xbeebuffer, " X%4d Y%4d Z%4d x%4d y%4d z%4d R%4d r%4d L%4d l%4d\n\r", rollcash[0], rollcash[1], rollcash[2], accelcash[0], accelcash[1], accelcash[2], motorr, servor, motorl, servol);
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
			for(i = 0; i < 3; i ++){
				accelcash[i] *= INTEGRATEACCEL;
				rollcash[i] *= INTEGRATEROLL;
			}
		}		
	

	break;

	}
	}

	
	return 0;

}

ISR(USARTE0_RXC_vect){
	USART_RXComplete(&IMU);
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&IMU);
}

ISR(USARTC0_RXC_vect){
	USART_RXComplete(&xbee);
}

ISR(USARTC0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}
