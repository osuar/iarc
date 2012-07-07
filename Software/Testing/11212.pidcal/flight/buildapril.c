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
	int testcounter = 0;

	int integration[3] = {0,0,0};

	char motordif24 = -15;	
	char motordif13 = 10;

	uint8_t testchar = 0;
	uint8_t testchar2 = 0;

	int pry[] = {0,0,0};
	int aVec[] = {0,0,0};

	int paceCounter = 0;


	int tilt = 0;

	int pidValues[3] = {14,0,28};
	int pidValuesDen[3] = {16,1,1};

	/*counting var, for for loops*/
	int i;


	/*Start memory location for Accel and Gyro reads, should be moved
	  to gyro and accel read functions*/
	uint8_t accelstartbyte = 0x30;
	uint8_t gyrostartbyte = 0x1A;

	/*Joystick Axis buffer
	  [0] - X axis tilt
	  [1] - Y axis tilt
	  [2] - Throttle
	  [3] - Rotation about Z axis
	 */
	char joyaxis[] = {0,0,0,0};
	int motorSpeeds[4];

	/*Var to allow increase in motor speed nonrelative to the throttle
	  during flight*/
	int motorup = 0;

	/*Vars for new input raw data (cache) and filtered data (int) from
	  imu*/
	int gyrocache[3] = {0,0,0};
	int accelcache[3] = {0,0,0};
	int accelint[] = {0, 0, 0};
	int gyroint[] = {0, 0, 0};
	int gyrocounter[] = {0,0,0};


	/*Standard values for accel and gyro (when level), set during offset*/
	int accelnorm[3] = {38,-4,470};
	int gyronorm[3] = {16,42,0};

	/*Buffer for sending data through the xbee*/
	char xbeebuffer[100];

	/*Initialize PORTD to output on pins 0-3 from Timer counter pwm at
	  50Hz*/
	PORTD.DIR = 0x2F;
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCCEN_bm |  TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 4000;

	/*Initialize Timer counter C0 for pacing,RATE Hz*/
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 2000000 / RATE;
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
			else if (input[0] == 'o'){
				state = offset;
			}
			else if (input[0] == 'u'){
				pidValues[2] ++;
				sprintf(xbeebuffer, "%d/%d\n\r", pidValues[2], pidValuesDen[2]);
				sendstring(&xbee, xbeebuffer);
			}
			else if (input[0] == 'j'){
				pidValues[2] --;
				sprintf(xbeebuffer, "%d/%d\n\r", pidValues[2], pidValuesDen[2]);
				sendstring(&xbee, xbeebuffer);
			}
			else if (input[0] == 'i'){
				pidValuesDen[2] ++;
				sprintf(xbeebuffer, "%d/%d\n\r", pidValues[2], pidValuesDen[2]);
				sendstring(&xbee, xbeebuffer);
			}
			else if (input[0] == 'k'){
				pidValuesDen[2] --;
				sprintf(xbeebuffer, "%d/%d\n\r", pidValues[2], pidValuesDen[2]);
				sendstring(&xbee, xbeebuffer);
			}
			else if (input[0] == 'd'){
				pidValues[0] ++;
				sprintf(xbeebuffer, "%d/%d\n\r", pidValues[0], pidValuesDen[0]);
				sendstring(&xbee, xbeebuffer);
			}
			else if (input[0] == 'c'){
				pidValues[0] --;
				sprintf(xbeebuffer, "%d/%d\n\r", pidValues[0], pidValuesDen[0]);
				sendstring(&xbee, xbeebuffer);
			}
			else if (input[0] == 'f'){
				pidValuesDen[0] ++;
				sprintf(xbeebuffer, "%d/%d\n\r", pidValues[0], pidValuesDen[0]);
				sendstring(&xbee, xbeebuffer);
			}
			else if (input[0] == 'v'){
				pidValuesDen[0] --;
				sprintf(xbeebuffer, "%d/%d\n\r", pidValues[0], pidValuesDen[0]);
				sendstring(&xbee, xbeebuffer);
			}


			else if(input[0] == 'y'){
				motorup += 25;
			}
			else if(input[0] == 'h'){
				motorup -=25;
			}

			else if(input[0] == 't'){
				motordif24 += 5;
			}
			else if(input[0] == 'g'){
				motordif24 -= 5;
			}
			else if(input[0] == 'a'){
				motordif13 += 5;
			}
			else if(input[0] == 'z'){
				motordif13 -= 5;
			}
			else if(input[0] == '0'){
				cli();
				while(1){
					TCD0.CCA = 2000;
					TCD0.CCB = 2000;
					TCD0.CCC = 2000;
					TCD0.CCD = 2000;
				}
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
			sprintf(xbeebuffer, "%4d %4d %4d %4d\n", joyaxis[0], joyaxis[1], joyaxis[2], joyaxis[3]);
			sendstring(&xbee, xbeebuffer);
			}
			else if(input[7] == 4){
			state = running;
			sprintf(xbeebuffer, "running %d\n", input[7]);		sendstring(&xbee, xbeebuffer);
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
					getgyro(gyrocache, &imu, &gyrostartbyte);
					getaccel(accelcache, &imu, &accelstartbyte);
				for(i = 0; i < 3; i ++){
					gyronorm[i] = gyrocache[i];
					accelnorm[i] = accelcache[i];
					accelcache[i] = 0;
					gyrocache[i] = 0;
				}

				sprintf(xbeebuffer, "offset %d %d %d %d %d %d\n\r", gyronorm[0], gyronorm[1], gyronorm[2], accelnorm[0], accelnorm[1], accelnorm[2]);
				sendstring(&xbee, xbeebuffer);
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

					if((gyrocache[i] <= 1) && (gyrocache[i] >= -1)){
						gyrocache[i] = 0;
					}

					gyrocounter[i] += gyrocache[i];
				}

				for(i = 0; i < 3; i ++){

					//	gyroint[i] = ((GYROINT)* gyroint[i]);
					//	gyroint[i] += (20 - GYROINT) * gyrocounter[i]/DEGREE;
					gyroint[i] = gyrocounter[i]/DEGREE;
					gyrocounter[i] %= DEGREE;

					//gyroint[i] /= 20;


					pry[i] += gyroint[i];

				}



				paceCounter ++;
				//Slower Operations at 50Hz

				if(paceCounter == (RATE / 50)){
				paceCounter = 0;

					getaccel(accelcache, &imu, &accelstartbyte);

					for(i = 0; i < 3; i ++){
						accelcache[i] -= accelnorm[i];
					}

						accelint[0] = -((ACCELINT * accelint[0]) + ((20 - ACCELINT) * accelcache[0]))/20;
						accelint[1] = ((ACCELINT * accelint[1]) + ((20 - ACCELINT) * accelcache[1]))/20;



					if(accelint[0] > (pry[1] + 10)){
						accelint[0] = pry[1] + 10;
					}
					else if(accelint[0] < (pry[1] - 10)){
						accelint[0] = pry[1] - 10;
					}



					if(accelint[1] > (pry[0] + 10)){
						accelint[1] = pry[0] + 10;
					}
					else if(accelint[1] < (pry[0] - 10)){
						accelint[1] = pry[0] - 10;
					}

					
					


					pry[0] = ((AWEIGHT * accelint[1]) + (GWEIGHT * pry[0])) / (AWEIGHT + GWEIGHT);

					pry[1] = ((AWEIGHT * accelint[0]) + (GWEIGHT * pry[1])) / (AWEIGHT + GWEIGHT);

/*
				_delay_ms(500);
				testcounter ++;
				sprintf(xbeebuffer, "%d \n \r", testcounter);
*/
				//sprintf(xbeebuffer, "%4d %4d %4d\n\r", gyrocache[0], gyrocache[1], gyrocache[2]);
				sprintf(xbeebuffer, "%d %d %d\n\r", pry[0], pry[1], pry[2]);
				//sprintf(xbeebuffer, "%d %d %d\n\r", accelint[0], accelint[1], accelint[2]);
				//sprintf(xbeebuffer, "%d %d %d\n\r", accelcache[0], accelcache[1], accelcache[2]);
				//sprintf(xbeebuffer, "%d %d %d\n\r", currentMatrix[0], currentMatrix[1], currentMatrix[2]);	
				//sprintf(xbeebuffer, "%3d %3d %3d\n\r", gyroint[0], gyroint[1], gyroint[2]);
				//sprintf(xbeebuffer, "%4d %4d %4d %4d\n\r", motorSpeeds[0], motorSpeeds[1], motorSpeeds[2], motorSpeeds[3]);
				//sprintf(xbeebuffer, "%d\n\r", gyrocache[1]);
				
				//sprintBinary(xbeebuffer, &gyrocache[0]);
				sendstring(&xbee, xbeebuffer);

				/*reset cache values to 0, should be made unnecessary by modding gyro and
				  accel read functions*/
				for(i = 0; i < 3; i ++){
					accelcache[i] = 0;
				}

				}



				/*Get accel data
				  substract accelnorm (should change that, won't work with dcm)
				  Filter
				 */



				motorSpeed(pry, integration ,gyroint, joyaxis, motorSpeeds, pidValues, pidValuesDen);


				TCD0.CCA = motorSpeeds[0] + motorup;// - motordif13;
				TCD0.CCC = motorSpeeds[2] + motorup;// +  motordif13;
				TCD0.CCB = motorSpeeds[1] + motorup;// + motordif24;
				TCD0.CCD = motorSpeeds[3] + motorup;// - motordif24;



				pry[2] = 0;

				PORTD.OUT ^= 0b00100000;


				/*
				   if((abs(pry[0]) > 80) || (abs(pry[1]) > 80)){
				   state = stopped;
				   }
				 */

				/*
				   for(i = 1; i < 10; i ++){
				   xbeebuffer[i] = currentMatrix[i - 1] + 62;
				   }
				   xbeebuffer[0] = 0xFB;
				   xbeebuffer[10] = 0xFF;
				   xbeebuffer[11] = 0;
				 */
		}

	}
}



	/*Xbee read interrupt*/
ISR(USARTE0_RXC_vect){
	USART_RXComplete(&xbee);
	input[xbeecounter] = USART_RXBuffer_GetByte(&xbee);

	/*For non joystick packets*/
	readdata = 1;
	/*q should reset device*/
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

