#include <stdlib.h>
#include "../../drivers/avr_compiler.h"
#include "../../drivers/usart_driver.h"
#include "../../drivers/twi_master_driver.h"
#include "support.h"
#include <stdio.h>
#include "itg3200.h"
#include "adxl345.h"
#include "math.h" 
#include "lsm303dlh.h"

/*Xbee Wireless Communication Module, initialized later*/
USART_data_t xbee;
USART_data_t mega;

/*Two wire interface module for Inertial Measurement Unit*/
TWI_Master_t imu;

/*States for flight control state machine*/
enum states{running, stopped, offset} state = stopped;

/*Flag for complete Xbee packet receival*/
volatile char readdata = 0;
volatile char readdatair = 0;
/*Packet buffer for incoming data from Xbee*/
volatile char input[11] = {0,0,0,0,0,0,0,0,0,0,0};
volatile char inputir[10] = {0,0,0,0,0,0,0,0,0,0};
/*Counter for packet position*/
volatile char xbeecounter = 0;
volatile char megacounter = 0;

int main(void){


	//integration[2] is yaw
	int integration[3] = {0,0,0};

	int mag_correct;

	int gyro_informer = 0;

	char lostsignalcnt = 0;

	int pry[] = {0,0,0};

	int paceCounter = 0;
	int pace_counter_40 = 0;
	int pace_counter_60 = 0;

	int pidValues13[3] = {11,0,22};
	int pidValuesDen13[3] = {16,1,1};

	int pidValues24[3] = {11,0,22};
	int pidValuesDen24[3] = {16,1,1};

	char pidRotUp[3] = {9,0,20};
	char pidRotDenUp[3] = {42,11,1};

	char pidRotDown[3] = {9,0,20};
	char pidRotDenDown[3] = {42,11,1};

	char pidRot[] = {18,0,40};

	/*counting var, for for loops*/
	int i;
	int j;

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
	int joyaxis[] = {0,0,0,0,0};
	char joyin[] = {0,0,0,0,0};
	int joytrim[] = {0,0,0,0,0};
	int joydif[] = {0,0};
	int joyavr[] = {0,0};
	int motorSpeeds[4];


	/*Vars for new input raw data (cache) and filtered data (int) from
	  imu*/
	int gyrocache[3] = {0,0,0};
	int accelcache[3] = {0,0,0};
	int magcache[3] = {0,0,0};
	int magfiltered[3] = {0,0,0};
	int magfacing = 0;
	int roterr = 0;
	int target[] = {0,0,0};
	int accelint[] = {0, 0, 0}; 
	int gyroint[] = {0, 0, 0}; 
	int gyrocounter[] = {0,0,0}; 
	char irdata[2] = {0,0};
	char ir_horizontal_1[2] = {0,0};

	//For offsets
	int acceloffsetcache[3];
	int gyrooffsetcache[3];
	int magoffsetcache;


	/*Standard values for accel and gyro (when level), set during offset*/
	int accelnorm[3] = {28,-20,468};
	char gyronorm[3] = {16,42,0};

	/*Buffer for sending data through the xbee*/
	char xbeebuffer[100];

	/*Byte for status update sent in data packet*/
	char stat;


	/*Set on board LED pins to output*/
	PORTF.DIR = 0x03;
	//	CLK.CTRL = 0b00000011;
	//	CLK.PSCTRL = 0b00010100;
	OSC.CTRL = 0b00000010;
	while(!(OSC.STATUS & 0b00000010));
	CCPWrite(&CLK.CTRL, 0b00000001);

	/*Initialize PORTD to output on pins 0-3 from Timer counter pwm at
	  50Hz*/
	PORTD.DIR = 0x2F;
	TCD0.CTRLA = TC_CLKSEL_DIV8_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCCEN_bm |  TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 8000;

	/*Initialize Timer counter C0 for pacing,RATE Hz*/
	TCC0.CTRLA = TC_CLKSEL_DIV8_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 4000000 / RATE;

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
	TCD0.CCA = 4000;
	TCD0.CCB = 4000;
	TCD0.CCC = 4000;
	TCD0.CCD = 4000;

	/*Set Xbee Uart transmit pin 3 to output*/
	PORTE.DIR = 0x88;
	/*Initialize USARTE0 as the module used by the Xbee*/
	uartInitiate(&xbee, &USARTE0, 115200);
	uartInitiate(&mega, &USARTE1, 9600);

	/*Initialize imu to use Two wire interface on portC*/
	twiInitiate(&imu, &TWIC);
	itg3200Init(&imu, RATE);
	adxl345Init(&imu);
	lsm303dlhInit(&imu);

	/*Send string to indicate startup, python doesn't like return carriage
	  (/r) character*/
	sprintf(xbeebuffer, "starting\n");
	sendstring(&xbee, xbeebuffer);

	stat = 9;

	/*Start of flight control state machine loop*/
	while(1){
		/*Check for new packet from xbee each time*/
		if(readdata){
			readdata = 0;
			lostsignalcnt = 0;


			/*For Joystick packet reads*/

			joytrim[2] = 0;
			for(i = 0; i < 5; i++){
				joyin[i] = input[3 + i] - 127;
				joyaxis[i] = joyin[i];
				joyaxis[i] += joytrim[i];
			}

			//throttleavr = ((throttleavr) + (joyaxis[2]))/2;
			//throttledif = joyaxis[2] - throttleavr;
			//joyaxis[2] += throttledif * THROTTLEJOYDIF;

			for(i = 0; i < 2; i++){
				joyavr[i] = (joyavr[i] + joyaxis[i])/2;
				joydif[i] = joyaxis[i] - joyavr[i];
			}

			joyaxis[1] += joydif[1] * PRJOYDIF13;
			joyaxis[0] += joydif[0] * PRJOYDIF24;



			/*
			   yawavr = ((yawavr) + joyaxis[3])/2;
			   yawdif = joyaxis[3] - yawavr;
			   joyaxis[3] += yawdif * YAWJOYDIF;
			 */
			//Input 7 is the button buffer
			if(input[8] == 4){
				state = stopped;
				stat = 4;
				PORTF.OUT ^= 1;
			}
			else if(input[8] == 0){
				joytrim[0] += joyin[0];
				joytrim[1] += joyin[1];
				joytrim[3] += joyin[3];
			}
			else if(input[8] == 1){
				state = running;
				stat = 1;
			}
			else if(input[8] == 10){
				state = offset;
				stat = 10;
			}
			else if(input[8] == 5){
				//pidRot[2] ++;
				
				   pidValues13[2] ++;
				   pidValues24[2] ++;
				 


			}
			else if(input[8] == 6){
				//pidRot[2] --;
				
				   pidValues13[2] --;
				   pidValues24[2] --;
				 
			}
			else if(input[8] == 7){
				//pidRot[0] ++;
				
				   pidValues13[0] ++;
				   pidValues24[0] ++;
				 
			}
			else if(input[8] == 8){

				//pidRot[0] --;
				
				   pidValues13[0] --;
				   pidValues24[0] --;
				 
			}
			xbeecounter = 0;

			for(i=0;i<3;i++){
				pidRotUp[i] = pidRot[i] * 3/4;
				pidRotDown[i] = pidRot[i] * 1;
			}

			//packet to send back		
			xbeebuffer[0] = 'v';
			xbeebuffer[1] = 'e';
			xbeebuffer[2] = 'x';
			xbeebuffer[3] = irdata[0];
			xbeebuffer[4] = irdata[1];


			/*	FOR General Use
			 */
			xbeebuffer[5] = pry[0] >> 8;
			xbeebuffer[6] = pry[0] & 0xff;
			xbeebuffer[7] = pry[1] >> 8;;
			xbeebuffer[8] = pry[1] & 0xff;
			xbeebuffer[9] = roterr >> 8;
			xbeebuffer[10] = roterr & 0xff;


			/*	For Checking Offsets

				xbeebuffer[5] = accelnorm[0] >> 8;
				xbeebuffer[6] = accelnorm[0] & 0xff;
				xbeebuffer[7] = accelnorm[1] >> 8;
				xbeebuffer[8] = accelnorm[1] & 0xff;
				xbeebuffer[9] = target[2] >> 8;
				xbeebuffer[10] = target[2] & 0xff;
			 */
			/*	For PID tuning

				xbeebuffer[5] = 0;
				xbeebuffer[6] = pidValues13[0];
				xbeebuffer[7] = 0;
				xbeebuffer[8] = pidValues13[2];;
				xbeebuffer[9] = roterr >> 8;
				xbeebuffer[10] = roterr & 0xff;
*/			 
			/*
			   xbeebuffer[5] = gyroint[2] >> 8;
			   xbeebuffer[6] = gyroint[2] & 0xff;;
			   xbeebuffer[7] = roterr >> 8;
			   xbeebuffer[8] = roterr & 0xff;
			   xbeebuffer[9] = joytrim[3] >> 8;
			   xbeebuffer[10] = joytrim[3] & 0xff;

			 */
			/*
			   xbeebuffer[5] = gyroint[0] >> 8;
			   xbeebuffer[6] = gyroint[0] & 0xff;;
			   xbeebuffer[7] = pry[0] >> 8;
			   xbeebuffer[8] = pry[0] & 0xff;
			   xbeebuffer[9] = pry[2] >> 8;
			   xbeebuffer[10] = pry[2] & 0xff;


			   xbeebuffer[5] = 0;
			   xbeebuffer[6] = joyaxis[3];
			   xbeebuffer[7] = 0;
			   xbeebuffer[8] = joyaxis[4];
			   xbeebuffer[9] = 0;
			   xbeebuffer[10] = joyaxis[0];

			 */
			xbeebuffer[11] = ir_horizontal_1[0];
			xbeebuffer[12] = ir_horizontal_1[1];
			xbeebuffer[13] = stat;
			xbeebuffer[14] = 'u';
			sendpacket(&xbee, xbeebuffer);

			//If IR data ready



		}
		if(readdatair == 1){
			readdatair = 0;
			//irdata = (inputir[1] << 8) | inputir[2];
			irdata[0] = inputir[1];
			irdata[1] = inputir[2];
			ir_horizontal_1[0] = inputir[3];
			ir_horizontal_1[1] = inputir[4];
			megacounter = 0;
		}
		switch(state){

			/*Stopped state keeps motors stopped but not beeping*/
			case stopped:
				TCD0.CCA = 4000;
				TCD0.CCB = 4000;
				TCD0.CCC = 4000;
				TCD0.CCD = 4000;
				break;
				/*Offset gets standard value for gyro's and accel's*/
			case offset:
				for(i=0;i<3;i++){
					acceloffsetcache[i] = 0;
					gyrooffsetcache[i] = 0;
				}

				magoffsetcache = 0;
				for(i =0; i<8;i++){
					getgyro(gyrocache, &imu, &gyrostartbyte);
					getaccel(accelcache, &imu, &accelstartbyte);
					getmag(magcache, &imu);
					for(j=0;j<3;j++){
						acceloffsetcache[j] += accelcache[j];
						gyrooffsetcache[j] += gyrocache[j];
					}
					magoffsetcache += arctan2(magcache[0], magcache[1]);
					_delay_ms(100);
				}
				for(i=0;i<3;i++){
					accelcache[i] = acceloffsetcache[i]/8;
					gyrocache[i] = gyrooffsetcache[i]/8;
				}
				target[2] = magoffsetcache/8;
				pry[2] = target[2];

				for(i = 0; i < 3; i ++){
					gyronorm[i] = gyrocache[i];
					accelnorm[i] = accelcache[i];
				}


				state = stopped;

				break;



			case running:
				/*Ensure loop doesn't go faster than 200Hz*/
				while(!(TCC0.INTFLAGS & 0x01));
				TCC0.INTFLAGS = 0x01;

				/*Get gyro data
				  subtract stationary offset
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


				gyroint[0] = (gyrocounter[0])/DEGREE;
				gyrocounter[0] %= DEGREE;
				pry[0] += gyroint[0];


				gyroint[2] = gyrocounter[2]/DEGREE;
				gyrocounter[2] %= DEGREE ;
				pry[2] += gyroint[2];
				gyro_informer += gyroint[2];
				if(pry[2] > PI){
					pry[2] = -PI + (pry[2] - PI);
				}
				else if(pry[2] < -PI){
					pry[2] = PI + (pry[2] + PI);
				}


				gyroint[1] = -(gyrocounter[1]/DEGREE);
				gyrocounter[1] %= DEGREE;
				pry[1] += gyroint[1];



				paceCounter ++;
				pace_counter_40 ++;
				pace_counter_60 ++;
				//For 60Hz
				if(pace_counter_60 == 3){
					pace_counter_60 = 0;
					//For IR data
					sendchar(&mega, 'r');
				}

				//For 40hz
				if(pace_counter_40 == (RATE / 40)){
					pace_counter_40 = 0;
					getmag(magcache, &imu);

					for(i = 0;i < 3; i++){
						magfiltered[i] = ((9 * magfiltered[i]) + (magcache[i]))/10;
					}

					magfacing = arctan2(magfiltered[0], magfiltered[1]);
				}


				//Slower Operations at 20Hz
				if(paceCounter == (RATE / 20)){
					paceCounter = 0;
					lostsignalcnt ++;

					getaccel(accelcache, &imu, &accelstartbyte);
					//pry[2] = ((11 * pry[2]) + (magfacing))/12;
					if((4900 - abs(pry[2]) - abs(magfacing)) < abs(pry[2] - magfacing)){

						mag_correct = (4900 - abs(magfacing) - abs(pry[2]))/12;
						if(magfacing < 0){
							if(mag_correct > 10){
								mag_correct = 10;
							}
							pry[2] += mag_correct;
						}
						else{
							if(mag_correct > 10){
								mag_correct = 10;
							}
							pry[2] -= mag_correct;
						}
					}
					else{
						mag_correct = (magfacing - pry[2])/12;
						if(mag_correct > 5){
							mag_correct = 5;
						}
						else if(mag_correct < -5){
							mag_correct = -5;
						}
						pry[2] += mag_correct;
					}


					if((4900 - abs(pry[2]) - abs(target[2])) < abs(pry[2] - target[2])){
						if(target[2] > 0){
							roterr = 4900 - abs(target[2]) - abs(pry[2]);
						}
						else{
							roterr = -(4900 - abs(target[2]) - abs(pry[2]));
						}
					}
					else{
						roterr = target[2] - pry[2];
					}

					if(roterr > 411){
						roterr = 411;
					}
					else if(roterr < -411){
						roterr = -411;
					}

					integration[2] += roterr/40;
					if(integration[2] > YAW_INT_MAX){
						integration[2] = YAW_INT_MAX;
					}
					else if(integration[2] < -YAW_INT_MAX){
						integration[2] = -YAW_INT_MAX;
					}


					for(i = 0; i < 3; i ++){
						accelcache[i] -= accelnorm[i];
						/*
						   if(accelcache[i] > (accelint[i] + 40)){
						   accelcache[i] = accelint[i] + 40;
						   }
						   else if(accelcache[i] < (accelint[i] - 40)){
						   accelcache[i] = accelint[i] - 40;

						   }
						 */
					}


					accelint[0] = ((ACCELINT * accelint[0]) + ((24 - ACCELINT) * accelcache[0]))/24;


					accelint[1] = ((ACCELINT * accelint[1]) + ((24 - ACCELINT) * accelcache[1]))/24;


					if(accelint[1] > (pry[0] + 15)){
						accelint[1] = pry[0] + 15;
					}
					else if(accelint[1] < (pry[0] - 15)){
						accelint[1] = pry[0] - 15;
					}


					if(accelint[0] > (pry[1] + 15)){
						accelint[0] = pry[1] + 15;
					}
					else if(accelint[0] < (pry[1] - 15)){
						accelint[0] = pry[1] - 15;
					}


					pry[0] = ((AWEIGHT * accelint[1]) + (GWEIGHT * pry[0])) / (AWEIGHT + GWEIGHT);

					pry[1] = ((AWEIGHT * accelint[0]) + (GWEIGHT * pry[1])) / (AWEIGHT + GWEIGHT);

					/*reset cache values to 0, should be made unnecessary by modding gyro and
					  accel read functions*/
					for(i = 0; i < 3; i ++){
						accelcache[i] = 0;
					}


				}

				if(gyroint[0] > 6){
					gyroint[0] = 6;
				}
				else if(gyroint[0] < -6){
					gyroint[0] = -6;
				}


				if(gyroint[1] > 6){
					gyroint[1] = 6;
				}
				else if(gyroint[1] < -6){
					gyroint[1] = -6;
				}


				motorSpeed(pry, integration ,gyroint, joyaxis, motorSpeeds, pidValues13, pidValues24, pidValuesDen13, pidValuesDen24);
				yawCorrect(motorSpeeds, gyroint, integration, &roterr,pidRotUp,pidRotDenUp,pidRotDown,pidRotDenDown);

				/*
				   if(lostsignalcnt > 10){
				   for(i = 0; i < 4; i ++){
				   motorSpeeds[i] -= 50;
				   }
				   }
				 */

				while(!((TCD0.CNT > 10000) || (TCD0.CNT < 5000)));

				TCD0.CCA = 2 * motorSpeeds[0];// - motordif13;

				TCD0.CCC = 2 * motorSpeeds[2];// +  motordif13;
				TCD0.CCB = 2 * motorSpeeds[1];// + motordif24;
				TCD0.CCD = 2 * motorSpeeds[3];// - motordif24;




				PORTD.OUT ^= 0b00100000;	


		}

	}
}



	/*Xbee read interrupt*/
ISR(USARTE0_RXC_vect){
	USART_RXComplete(&xbee);
	input[xbeecounter] = USART_RXBuffer_GetByte(&xbee);

	

	/*For joystick packets*/
	if((input[0] == ' ') && (xbeecounter == 0)){
		xbeecounter ++;



	}
	else if((input[1] == '0') && (xbeecounter == 1)){
	
		xbeecounter ++;
	}
	else if((input[2] == 'a') && (xbeecounter == 2)){

		xbeecounter ++;
	}
	else if((xbeecounter >= 3) && (xbeecounter <= 8)){
		xbeecounter ++;

	}

	else if((input[9] == 's') && (xbeecounter == 9)){
		readdata = 1;


		if(input[8] == 9){

			CCPWrite( &RST.CTRL, 1 );
		}

		xbeecounter ++;
	}

	else if((input[xbeecounter] == ' ')){

		xbeecounter = 1;
	}
	else if((input[xbeecounter] == '0')){

		xbeecounter = 2;
	}
	else if((input[xbeecounter] == 'a')){
		xbeecounter = 3;
	}


}

/*Usart module interrupt to inform data has been properly sent*/
ISR(USARTE0_DRE_vect){
	USART_DataRegEmpty(&xbee);
}

ISR(USARTE1_RXC_vect){
	USART_RXComplete(&mega);
	inputir[megacounter] = USART_RXBuffer_GetByte(&mega);


	/*For joystick packets*/
	if((inputir[0] == 'a') && (megacounter == 0)){


		megacounter ++;
	}
	else if((megacounter >= 1) && (megacounter <= 8)){
		megacounter ++;

	}

	else if((inputir[9] == 'z') && (megacounter == 9)){
		readdatair = 1;
		megacounter ++;
		PORTF.OUT ^= 1;

	}

	else if((inputir[megacounter] == 'a')){
		megacounter = 1;
	}


}

/*Usart module interrupt to inform data has been properly sent*/
ISR(USARTE1_DRE_vect){
	USART_DataRegEmpty(&mega);
}



/*Inertial measurement unit interrupt support routine, could be implemented by polling*/
ISR(TWIC_TWIM_vect){
	TWI_MasterInterruptHandler(&imu);
}

