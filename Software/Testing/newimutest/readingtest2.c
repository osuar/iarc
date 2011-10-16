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



TWI_Master_t imu;

USART_data_t xbee;

volatile int readdata = 0;

int main(void){
	int i;
	int bytetobuffer;
	char xbeebuffer[100];
	char receive;
	int count = 0;

	uint8_t accelsetupbuffer1[3] = {0x2C, 0b00001100, 0x08};
	uint8_t accelsetupbuffer2[3] = {0x31, 0x00};
	uint8_t accelstartbyte = 0x30;
	uint8_t rollsetupbuffer1[4] = {0x15, 0x04, 0x19, 0x11};
	uint8_t rollsetupbuffer2[] = {0x3E, 0b00000001};
	uint8_t rollstartbyte = 0x1A;
	
	char rollcash[3] = {0,0,0};
	int accelcash[3] = {0,0,0};

	short int motorr = 0;
	short int motorl = 0;
	short int servor = 0;
	short int servol = 0;

	enum states {runningread, stopped} state = stopped;

	/**Setup directions for serial interfaces*/
	PORTC.DIR = 0b00001100;
	PORTC.OUT |= 0b00001000;
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
	TWI_MasterInit(&imu, &TWIC, TWI_MASTER_INTLVL_HI_gc, TWI_BAUDSETTING);

	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ACCEL, accelsetupbuffer1, 3, 0);
	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ACCEL, accelsetupbuffer2, 2, 0);
	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer1, 4, 0);
	while(imu.status != TWIM_STATUS_READY);
	TWI_MasterWriteRead(&imu, ROLL, rollsetupbuffer2, 2, 0);
	while(imu.status != TWIM_STATUS_READY);

	/**Setup Xbee*/
	USART_InterruptDriver_Initialize(&xbee, &USARTE0, USART_DREINTLVL_LO_gc);
	USART_Format_Set(xbee.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	USART_RxdInterruptLevel_Set(xbee.usart, USART_RXCINTLVL_HI_gc);
	USART_Baudrate_Set(&USARTE0, 12 , 0);
	USART_Rx_Enable(xbee.usart);
	USART_Tx_Enable(xbee.usart);

	while(1){


	if(USART_RXBufferData_Available(&xbee)){
		receive = USART_RXBuffer_GetByte(&xbee);
		if(receive == 'r'){
			state = runningread;
			sprintf(xbeebuffer, "read\n\r");
			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
		}
		else if(receive == 's'){
			state = stopped; 
			sprintf(xbeebuffer, "stop\n\r");
			for(i = 0; i < 3; i ++){
				accelcash[i] = 0;
				rollcash[i] = 0;
			}
			
			TCD0.CCA = 2000;
			TCD0.CCB = SERVOINI;
			TCD0.CCC = 2000;
			TCD0.CCD = SERVOINI;

			for(i = 0; xbeebuffer[i] != 0; i ++){
				bytetobuffer = 0;
				while(!bytetobuffer){
					bytetobuffer = USART_TXBuffer_PutByte(&xbee, xbeebuffer[i]);
				}
			}
		}
		else if(receive == 'q'){
			CCPWrite( &RST.CTRL, 1 );
			while(1);
		}
	}

	switch(state){

	case stopped:

	break;	

	case runningread:

		PORTF.OUT = 0;
		if(TCC0.INTFLAGS & 0x01){
			TCC0.INTFLAGS = 0x01;
			do{
				while(imu.status != TWIM_STATUS_READY);
				TWI_MasterWriteRead(&imu, ROLL, &rollstartbyte, 1, 10);
				while(imu.result == TWIM_RESULT_UNKNOWN);
				readdata = 0;
			}while(!(imu.readData[0] & 0x01));
			for(i = 0; i < 5; i += 2){
				rollcash[i/2] += ((char)(imu.readData[i + 3]));
			}

			PORTF.OUT = 1;
			do{
				while(imu.status != TWIM_STATUS_READY);
				TWI_MasterWriteRead(&imu, ACCEL, &accelstartbyte, 1, 10);
				while(imu.result == TWIM_RESULT_UNKNOWN);
				readdata = 0;
			}while(!(imu.readData[0] & 0x80));

			for(i = 0; i < 5; i += 2){
				if(imu.readData[i + 3] & 0x80){
					accelcash[i/2] -= 256 * (~imu.readData[i + 3] + 1);
					accelcash[i/2] -= ~imu.readData[i + 2] + 1;
				}
				else{
					accelcash[i/2] += 256 * imu.readData[i + 3];
					accelcash[i/2] += imu.readData[i + 2];
				}
			}
		}

		count ++;
		if(count > 4){
			PORTF.OUT = 3;
			for(i = 0; i < 3; i ++){
				accelcash[i] /= DAMPENACCEL;
				rollcash[i] /= DAMPENROLL;
			}


		//motor updates
			rollcash[0] -= RXN;
			rollcash[1] -= RYN;
			rollcash[2] -= RZN;
			accelcash[0] -= AXN;
			accelcash[1] -= AYN;
			accelcash[2] -= AZN;

			if(accelcash[0] >= 200){
				for(i = 0; i < 3; i ++){
					rollcash[i] = 0;
					accelcash[i] = 0;
				}
			}


			ValueFunk(accelcash[0],accelcash[1],accelcash[2],rollcash[0],rollcash[1],rollcash[2],&servol,&servor,&motorl,&motorr);
			while(TCD0.CNT < 4000);

			TCD0.CCA = motorr;
			TCD0.CCB = servor;
			TCD0.CCC = motorl;
			TCD0.CCD = servol;

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
	USART_RXComplete(&xbee);
}

ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}

ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&imu)
}	 

