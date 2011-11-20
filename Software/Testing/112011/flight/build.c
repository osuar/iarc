#include <stdlib.h>
#include "../../drivers/avr_compiler.h"
#include "../../drivers/usart_driver.h"
#include "../../drivers/twi_master_driver.h"
#include "support.h"
#include <stdio.h>
#include "dcm.h"

/*Xbee Wireless Communication Module, initialized later*/
USART_data_t xbee;

/*Two wire interface module for Inertial Measurement Unit*/
TWI_Master_t imu;

/*States for flight control state machine*/
enum states{running, stopped, offset} state = stopped;

/*Flag for complete Xbee packet receival*/
volatile char readdata = 0;
/*Packet buffer for incoming data from Xbee*/
volatile char input[9] = {0,0,0,0,0,0,0,0,0};
/*Counter for packet position*/
volatile char xbeecounter = 0;

int main(void){

	int updateMatrix[9];
	int currentMatrix[9];
	dcmInit(currentMatrix);
	
	/*counting var, for for loops*/
	int i;

	/*Start memory location for Accel and Gyro reads, should be moved
	  to gyro and accel read functions*/
	uint8_t accelstartbyte = 0x30;
	uint8_t gyrostartbyte = 0x1A;

	/*Joystick Axis buffer
	  [0] -
	  [1] -
	  [2] -
	  [3] -
	 */
	char joyaxis[] = {0,0,0,0};

	/*Var to allow increase in motor speed nonrelative to the throttle
	  during flight*/
	int motorup = 0;

	/*Vars for new input raw data (cache) and filtered data (int) from
	  imu*/
	char gyrocache[3] = {0,0,0};
	int accelcache[3] = {0,0,0};
	int accelint[] = {0, 0, 0};
	int gyroint[] = {0, 0, 0};

	/*support vars for offset function, should be moved to offset function*/
	char rolhisx[50];
	char rolhisy[50];
	char rolhisz[50];
	int acchisx[50];
	int acchisy[50];
	int acchisz[50];

	/*var for offset to indicate whether process has been completed
	  should be made relavent or removed*/
	char readyset = 0;

	/*Standard values for accel and gyro (when level), set during offset*/
	int accelnorm[3] = {0,0,0};
	char gyronorm[3] = {0,0,0};

	/*Buffer for sending data through the xbee*/
	char xbeebuffer[100];

	/*Initialize PORTD to output on pins 0-3 from Timer counter pwm at
	  50Hz*/
	PORTD.DIR = 0x0F;
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCCEN_bm |  TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 40000;

	/*Initialize Timer counter C0 for pacing, 50 Hz*/
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 40000;
	/*Set on board LED pins to output*/
	PORTF.DIR = 0x03;

	/*Set PORTC to power IMU, PIN 3 3.3V, pin 2 ground*/
	PORTC.DIR = 0b00001100;
	PORTC.OUT = 0b00001000;

	/*Enable global interrupts of all priority levels, should be made
	  more relevant*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
		PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	/*Set pwm duty cycle to stop motors, stop them from beeping 
	  annoyingly*/
	TCD0.CCA = 2000;
	TCD0.CCB = 2000;
	TCD0.CCC = 2000;
	TCD0.CCD = 2000;


	/*Set Xbee Uart transmit pin 3 to output*/
	PORTE.DIR = 0x08;
	/*Initialize USARTE0 as the module used by the Xbee*/
	uartInitiate(&xbee, &USARTE0);

	/*Initialize imu to use Two wire interface on portC*/
	twiInitiate(&imu, &TWIC);

	/*Send string to indicate startup, python doesn't like return carriage
	  (/r) character*/
	sprintf(xbeebuffer, "starting\n");
	sendstring(&xbee, xbeebuffer);

	/*Start of flight control state machine loop*/
	while(1){
		/*Check for new packet from xbee each time*/
		if(readdata){
			readdata = 0;
			/*For non joystick control*/

			if (input[0] == 'r') {
				state = running;
				sprintf(xbeebuffer, "running\r\n");
				sendstring(&xbee, xbeebuffer);
			}
			else if (input[0] == 's') {
				state =stopped;
				sprintf(xbeebuffer, "stopped\r\n");
				sendstring(&xbee, xbeebuffer);
			}


			/*For Joystick packet reads
			for(i = 0; i < 4; i ++){
				joyaxis[i] = input[3 + i] - 126;
			}
			//Input 7 is the button buffer
			if(input[7] == 3){
				state = stopped;
				sprintf(xbeebuffer, "stopped %d\n", input[7]);
				//sprintf(xbeebuffer, "%4d %4d %4d %4d\n\r", motor1, motor2, motor3, motor4);
				sendstring(&xbee, xbeebuffer);
			}
			else if(input[7] == 4){
				state = running;
				sprintf(xbeebuffer, "running %d\n", input[7]);
				sendstring(&xbee, xbeebuffer);
			}
			else if(input[7] == 5){
				state = offset;
				sprintf(xbeebuffer, "offsetting\n");
				sendstring(&xbee, xbeebuffer);
			}
			else if(input[7] == 6){
				motorup += 25;
			}
			else if(input[7] == 7){
				motorup -= 25;
			}
			xbeecounter = 0;
			*/
		}

		switch(state){

			/*Stopped state keeps motors stopped but not beeping*/
			case stopped:
				TCD0.CCA = 2000;
				TCD0.CCB = 2000;
				TCD0.CCC = 2000;
				TCD0.CCD = 2000;
				break;
			/*Offset gets standard value for gyro's and accel's*/
			case offset:
				updateoffset(&imu,
						accelnorm,
						gyronorm,
						rolhisx,
						rolhisy,
						rolhisz,
						acchisx,
						acchisy,
						acchisz,
						accelcache,
						gyrocache, 
						&readyset, 
						&gyrostartbyte, 
						&accelstartbyte);
				state = stopped;

				break;



			case running:
				/*Ensure loop doesn't go faster than 50Hz*/
				while(!(TCC0.INTFLAGS & 0x01));
				TCC0.INTFLAGS = 0x01;

				/*Get gyro data
				  substract stationary offset
				  filter for stability
				 */
				getgyro(gyrocache, &imu, &gyrostartbyte);

				for(i = 0; i < 3; i ++){
					gyrocache[i] -= gyronorm[i];
				}

				for(i = 0; i < 3; i ++){
					gyroint[i] = ((10 * gyroint[i]) + (10 * gyrocache[i]))/20;
				}

				/*Get accel data
				  substract level (should change that, won't work with dcm)
				  Filter
				 */
				getaccel(accelcache, &imu, &accelstartbyte);
				for(i = 0; i < 3; i ++){
					accelcache[i] -= accelnorm[i];
				}

				for(i = 0; i < 3; i ++){
					accelint[i] = ((16 * accelint[i]) + (4 * accelcache[i]))/20;
				}

				/*dcm  matrix test stuff*/
				updateMatrix[0] = 0;
				updateMatrix[1] = gyroint[2];
				updateMatrix[2] = -gyroint[1];
				updateMatrix[3] = -gyroint[2];
				updateMatrix[4] = 0;
				updateMatrix[5] = gyroint[0];
				updateMatrix[6] = gyroint[1];
				updateMatrix[7] = -gyroint[0];
				updateMatrix[8] = 0;

				mMultiply(updateMatrix, updateMatrix, currentMatrix);

				for(i = 0; i < 3; i ++){
					vectorScale(&currentMatrix[i * 3], CURRENTWEIGHT, &currentMatrix[i * 3]); 
				}
				mAdd(currentMatrix, currentMatrix, updateMatrix);
				

				mNormalize(currentMatrix, currentMatrix);
				orthoNormalize(currentMatrix);


				for(i = 1; i < 10; i ++){
					xbeebuffer[i] = currentMatrix[i];
				}
				xbeebuffer[0] = 0xFF;
				xbeebuffer[10] = 0;
				sendstring(&xbee, xbeebuffer);
				//sprintf(xbeebuffer, "%d %d %d\n\r", currentMatrix[0], currentMatrix[1], currentMatrix[2]);	
				//sprintf(xbeebuffer, "%d %d %d\n\r", negFlag[0], negFlag[1], negFlag[2]);
				//sprintf(xbeebuffer,"The square root of  is %d\n\r", mySqrt(24309));  
				sendstring(&xbee, xbeebuffer);

				/*reset cache values to 0, should be made unnecessary by modding gyro and
					accel read functions*/
				for(i = 0; i < 3; i ++){
					accelcache[i] = 0;
					gyrocache[i] = 0;
				}
			}
		}
	}



	/*Xbee read interrupt*/
	ISR(USARTE0_RXC_vect){
	USART_RXComplete(&xbee);
	input[xbeecounter] = USART_RXBuffer_GetByte(&xbee);

	/*For non joystick packets*/
	readdata = 1;
	//q should reset device
	if(input[0] == 'q'){
		CCPWrite( &RST.CTRL, 1 );
	}


	/*For joystick packets
	if((input[0] == ' ') && (xbeecounter == 0)){
		xbeecounter ++;
	}
	else if((input[1] == 's') && (xbeecounter == 1)){
		xbeecounter ++;
	}
	else if((input[2] == 'a') && (xbeecounter == 2)){
		xbeecounter ++;
	}
	else if((xbeecounter >= 3) && (xbeecounter <= 7)){
		xbeecounter ++;
	}
	else if((input[8] == 'r') && (xbeecounter == 8)){
		readdata = 1;
		PORTF.OUT ^= 0x01;
		if(input[7] == 1){
			CCPWrite( &RST.CTRL, 1 );
		}
		xbeecounter ++;
	}
	*/



}

/*Usart module interrupt to inform data has been properly sent*/
ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}

/*Inertial measurement unit interrupt support routine, could be implemented by polling*/
ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&imu);
}

