#include <stdlib.h>
#include "../../drivers/avr_compiler.h"
#include "../../drivers/usart_driver.h"
#include "../../drivers/twi_master_driver.h"
#include "support.h"
#include <stdio.h>
#include "itg3200.h"
#include "adxl345.h"
#include "math.h" 
#include "../../drl/adc.h"
//#include "lsm303dlh.h" 
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
volatile char inputir[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
/*Counter for packet position*/
volatile char xbeecounter = 0;
volatile char megacounter = 0;

int main(void){

	//Make the clock 32Mhz
	OSC.CTRL = 0b00000010;//Enable oscilator
	while(!(OSC.STATUS & 0b00000010));//wait for it to be ready
	CCPWrite(&CLK.CTRL, 0b00000001);//Enable oscilator

	//integration[2] is yaw
	int integration[3] = {0,0,0};

	int mag_correct;

	char lostsignalcnt = 0;

	int pry[] = {0,0,0};

	int paceCounter = 0;
	int pace_counter_60 = 0;

	int pidValues13[3] = {15,0,60};
	int pidValuesDen13[3] = {16,1,1};

	int pidValues24[3] = {15,0,60};
	int pidValuesDen24[3] = {16,1,1};

	char pidRotUp[3] = {11,0,20};
	char pidRotDenUp[3] = {42,11,1};

	char pidRotDown[3] = {11,0,20};
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
	int motorSpeeds[4];


	/*Vars for new input raw data (cache) and filtered data (int) from
	  imu*/
	int gyrocache[3] = {0,0,0};
	int accelcache[3] = {0,0,0};
	int magcache[3] = {0,0,0};
	int magfacing = 0;
	int roterr = 0;
	int target[] = {0,0,0};
	int accelint[] = {0, 0, 0}; 
	int gyroint[] = {0, 0, 0}; 
	int gyrocounter[] = {0,0,0}; 
	//int gyro_track  = 0;
	char irdata[2] = {0,0};
	char ir_horizontal_1[2] = {0,0};

	//offset buffers
	int gyrooffsetcache[3];
	int acceloffsetcache[3];
	int magoffsetcache = 0;
	int offset_counter = 0;

	/*Standard values for accel and gyro (when level), set during offset*/
	int accelnorm[3] = {11,-44,468};
	char gyronorm[3] = {16,42,0};

	int servo_angle = 0;
	int servo_angle_right = 0;
	int servo_angle_left = 0;
	int forward_servo_angle = 0;

	/*Buffer for sending data through the xbee*/
	char xbeebuffer[100];

	/*Byte for status update sent in data packet*/
	char stat;

	/*Set on board LED pins to output*/
	PORTF.DIR = 0x03;

	PORTA.DIR = 0;
	int distance_data[5] = {0,0,0,0,0};
	int distance_data_buffer[5] = {0,0,0,0,0};
	adcInit(&ADCA);

	/*Initialize PORTD to output on pins 0-3 from Timer counter pwm at
	  50Hz*/
	PORTD.DIR = 0x0F;
	TCD0.CTRLA = TC_CLKSEL_DIV8_gc;
	TCD0.CTRLB = TC_WGMODE_SS_gc | TC0_CCCEN_bm |  TC0_CCAEN_bm |TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 8000;


	/*Initialize PORTC to output on pins 4-5 from Timer counter pwm at
	  50Hz for horizontal sensors*/
	PORTC.DIR = 0x30;
	TCC1.CTRLA = TC_CLKSEL_DIV64_gc;
	TCC1.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm |TC0_CCBEN_bm;
	TCC1.PER = 10000;
	servo_angle_left = LEFT_SERVO_DEFAULT;
	servo_angle_right = RIGHT_SERVO_DEFAULT;
	TCC1.CCA = servo_angle_right;
	TCC1.CCB = servo_angle_left;

	/*Initialize PORTF to output on pin 0 from Timer counter pwm at
	  50Hz for forward sensors including camera*/
	PORTF.DIR = 0x03;
	TCF0.CTRLA = TC_CLKSEL_DIV64_gc;
	TCF0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm;
	TCF0.PER = 10000;

	forward_servo_angle = FORWARD_SERVO_DEFAULT;
	TCF0.CCA = forward_servo_angle;




	/*Set pwm duty cycle to stop motors, stop them from beeping 
	  annoyingly*/
	TCD0.CCA = 4000;
	TCD0.CCB = 4000;
	TCD0.CCC = 4000;
	TCD0.CCD = 4000;

	/*Initialize Timer counter C0 for pacing,RATE Hz*/
	TCC0.CTRLA = TC_CLKSEL_DIV8_gc;
	TCC0.CTRLB = TC_WGMODE_SS_gc;
	TCC0.PER = 4000000 / RATE;

	/*Set PORTC to power IMU, PIN 3 3.3V, pin 2 ground*/
	//PORTC.DIR = 0b00001000;
	//PORTC.OUT = 0b00001000;


	/*Enable Watch Dog Timer to save us from catastrophe*/
	CCPWrite( &WDT.CTRL, 0b00010111);

	/*Enable global interrupts of all priority levels, should be made
	  more relevant*/
	PMIC.CTRL |= PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm | PMIC_HILVLEX_bm |
		PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	/*Initialize imu to use Two wire interface on portC*/
	PORTE.PIN0CTRL |= 0b00011000;
	PORTE.PIN1CTRL |= 0b00011000;
	twiInitiate(&imu, &TWIE);
	itg3200Init(&imu, RATE);
	adxl345Init(&imu);
	//lsm303dlhInit(&imu);



	/*Set Xbee Uart transmit pin 3 to output*/
	PORTE.DIR = 0x88;
	/*Initialize USARTE0 as the module used by the Xbee*/
	uartInitiate(&xbee, &USARTE0, 115200);
	uartInitiate(&mega, &USARTE1, 9600);

	/*Send string to indicate startup, python doesn't like return carriage
	  (/r) character*/
	sprintf(xbeebuffer, "starting\n");
	sendstring(&xbee, xbeebuffer);

	/*Feed watchdog just in case*/
	asm("wdr");

	/*Set status byte to booting so base station knows what's going on*/
	stat = 9;

	/*Start of flight control state machine loop*/
	while(1){
		/*Check for new packet from xbee each time*/
		if(readdata){
			readdata = 0;
			lostsignalcnt = 0;


			/*For Base Station packet reads*/

			joytrim[2] = 0;
			for(i = 0; i < 5; i++){
				joyin[i] = input[3 + i] - 127;
				joyaxis[i] = joyin[i];
				joyaxis[i] += joytrim[i];
			}

			//Input 7 is the button buffer
			if(input[8] == 4){
				state = stopped;
				//sendchar(&mega, 'd');
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
				sendchar(&mega, 0);
			}
			else if(input[8] == 10){
				state = offset;
				stat = 10;
				offset_counter = 0;
			}
			else if(input[8] == 5){
				//pidRot[2] ++;
				//pidValues13[2] ++;
				//pidValues24[2] ++;
				servo_angle_left += 100;
			}
			else if(input[8] == 6){
				//pidRot[2] --;
				//pidValues13[2] --;
				//pidValues24[2] --;
				servo_angle_left -= 100;
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
			xbeebuffer[3] = distance_data[0] >> 8;//irdata[0];
			xbeebuffer[4] = distance_data[0] & 0xff;//irdata[1];


			/*	FOR General Use
*/		 
			xbeebuffer[5] = pry[0] >> 8;
			xbeebuffer[6] = pry[0] & 0xff;
			xbeebuffer[7] = pry[1] >> 8;;
			xbeebuffer[8] = pry[1] & 0xff;
			xbeebuffer[9] = pry[2] >> 8;
			xbeebuffer[10] = pry[2] & 0xff;


/*
			xbeebuffer[5] = motorSpeeds[0] >> 8;
			xbeebuffer[6] = motorSpeeds[0] & 0xff;
			xbeebuffer[7] = motorSpeeds[1] >> 8;;
			xbeebuffer[8] = motorSpeeds[1] & 0xff;
			xbeebuffer[9] = motorSpeeds[2] >> 8;
			xbeebuffer[10] = motorSpeeds[2] & 0xff;
*/

			/*	For Checking Offsets

				xbeebuffer[5] = gyrocache[0] >> 8;
				xbeebuffer[6] = gyrocache[0] & 0xff;
				xbeebuffer[7] = gyrocache[1] >> 8;
				xbeebuffer[8] = gyrocache[1] & 0xff;
				xbeebuffer[9] = gyrocache[2] >> 8;
				xbeebuffer[10] = gyrocache[2] & 0xff;
*/			 			 
			/*	For PID tuning

				xbeebuffer[5] = 0;
				xbeebuffer[6] = pidValues13[0];
				xbeebuffer[7] = 0;
				xbeebuffer[8] = pidValues13[2];;
				xbeebuffer[9] = pry[1] >> 8;
				xbeebuffer[10] = pry[1] & 0xff;
*/	 			 
			/*

			   xbeebuffer[5] = target[2] >> 8;
			   xbeebuffer[6] = target[2] & 0xff;;
			   xbeebuffer[7] = pry[2] >> 8;
			   xbeebuffer[8] = pry[2] & 0xff;
			   xbeebuffer[9] = magfacing >> 8;
			   xbeebuffer[10] = magfacing & 0xff;
			 
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
			
			xbeebuffer[11] = distance_data[1] >> 8;
			xbeebuffer[12] = distance_data[1] & 0xff;
			
			xbeebuffer[13] = distance_data[2] >> 8;
			xbeebuffer[14] = distance_data[2] & 0xff;

			xbeebuffer[15] = distance_data[3] >> 8;
			xbeebuffer[16] = distance_data[3] & 0xff;

			xbeebuffer[17] = distance_data[4] >> 8;
			xbeebuffer[18] = distance_data[4] & 0xff;
		
			xbeebuffer[19] = servo_angle_left >> 8;
			xbeebuffer[20] = servo_angle_left & 0xff;

			//xbeebuffer[11] = motorSpeeds[3] >> 8;
			//xbeebuffer[12] = motorSpeeds[3] & 0xff;

			//xbeebuffer[11] = gyro_track >> 8;
			//xbeebuffer[12] = gyro_track & 0xff;

			xbeebuffer[21] = stat;
			xbeebuffer[22] = 'u';

			asm("wdr");
			sendpacket(&xbee, xbeebuffer);

			TCF0.CCA = forward_servo_angle;
			TCC1.CCA = servo_angle_right;
			TCC1.CCB = servo_angle_left;
		}

		//For getting IR packets from the mega
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
			case offset:
				//Make it 20Hz
				for(i=0;i<(RATE/20);i++){
					while(!(TCC0.INTFLAGS & 0x01));
					TCC0.INTFLAGS = 0x01;
				}

				if(offset_counter == 0){	
					for(i=0;i<3;i++){
						gyrooffsetcache[i] = 0;
					}
					magoffsetcache = 0;
				}

				getgyro(gyrocache, &imu, &gyrostartbyte);
				getaccel(accelcache, &imu, &accelstartbyte);
				//getmag(magcache, &imu);
				for(j=0;j<3;j++){
					gyrooffsetcache[j] = ((gyrooffsetcache[j]) + (gyrocache[j]))/2;
					acceloffsetcache[j] = ((4 * acceloffsetcache[j]) + (accelcache[j]))/5;
				}
				magoffsetcache = ((9 * magoffsetcache) + (1 * arctan2(magcache[0], magcache[1])))/10;
				if(offset_counter == 50){
					for(i=0;i<3;i++){
						gyrocache[i] = gyrooffsetcache[i];
						accelcache[i] = acceloffsetcache[i];
					}
					target[2] = 0;//magoffsetcache;
					pry[2] = target[2];

					for(i = 0; i < 3; i ++){
						gyronorm[i] = gyrocache[i];
						accelnorm[i] = accelcache[i];
					}
					state = stopped;
				}
				else{
					offset_counter ++;
				}
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

					if(i == 2){
						if(gyrocache[i] < 0){
					//		gyrocache[i] = (gyrocache[i] * 17)/18;
						}
					}
					gyrocache[i] = gyrocache[i] >> 0;

					gyrocounter[i] += gyrocache[i];
				}


				gyroint[0] = (gyrocounter[0])/DEGREE;
				gyrocounter[0] %= DEGREE;
				pry[0] += gyroint[0];


				gyroint[2] = gyrocounter[2]/DEGREE;
				//gyro_track += gyroint[2];
				gyrocounter[2] %= DEGREE ;
				pry[2] += gyroint[2];
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
				pace_counter_60 ++;
				//For 60Hz
				if(pace_counter_60 == 3){
					pace_counter_60 = 0;
					//For IR data
				//	sendchar(&mega, 'r');
				}

				
				adcRead(&ADCA, distance_data_buffer);
				if(distance_data_buffer[0] > 550){
					distance_data_buffer[0] = distance_data[0];
				}
				for(i = 0; i < 5; i++){
					distance_data[i] = ((4 * distance_data[i]) + distance_data_buffer[i])/5;
				}

				//Slower Operations at 20Hz
				if(paceCounter == (RATE / 20)){
					paceCounter = 0;
					lostsignalcnt ++;

					// Magnetometer stuff
					if((abs(pry[0]) < 50) && (abs(pry[1]) < 50)){
						//getmag(magcache, &imu);
						magfacing = arctan2(magcache[0], magcache[1]);
					}

					if((4900 - abs(pry[2]) - abs(magfacing)) < abs(pry[2] - magfacing)){

						mag_correct = (4900 - abs(magfacing) - abs(pry[2]))/30;
						if(magfacing < 0){
							if(mag_correct > 3){
								mag_correct = 3;
							}
							//pry[2] += mag_correct;
						}
						else{
							if(mag_correct > 3){
								mag_correct = 3;
							}
							//pry[2] -= mag_correct;
						}
					}
					else{
						mag_correct = (magfacing - pry[2])/30;
						if(mag_correct > 3){
							mag_correct = 3;
						}
						else if(mag_correct < -3){
							mag_correct = -3;
						}
					//	pry[2] += mag_correct;
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


					//Accelerometer Stuff
					getaccel(accelcache, &imu, &accelstartbyte);
					for(i = 0; i < 3; i ++){
						accelcache[i] -= accelnorm[i];
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


				}

				//Cap gyro values so they don't mess up pd, probably not necessary
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


				if(lostsignalcnt > 12){
					for(i = 0; i < 4; i ++){
						motorSpeeds[i] = 4000;
						state = stopped;
					}
				}


				while(!((TCD0.CNT > 10000) || (TCD0.CNT < 5000)));

				TCD0.CCA = 2 * motorSpeeds[0];// - motordif13;
				TCD0.CCC = 2 * motorSpeeds[2];// +  motordif13;
				TCD0.CCB = 2 * motorSpeeds[1];// + motordif24;
				TCD0.CCD = 2 * motorSpeeds[3];// - motordif24;

				PORTC.OUT ^= 8;
				break;

		}

	}
}



	/*Xbee read interrupt*/
ISR(USARTE0_RXC_vect){
	USART_RXComplete(&xbee);
	input[xbeecounter] = USART_RXBuffer_GetByte(&xbee);

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

/*Mega read interrupt*/
ISR(USARTE1_RXC_vect){
	USART_RXComplete(&mega);
	inputir[megacounter] = USART_RXBuffer_GetByte(&mega);

	if((inputir[0] == 'a') && (megacounter == 0)){
		megacounter ++;
	}
	else if((megacounter >= 1) && (megacounter <= 8)){
		megacounter ++;
	}

	else if((inputir[9] == 'z') && (megacounter == 9)){
		readdatair = 1;
		megacounter ++;
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
ISR(TWIE_TWIM_vect){
	TWI_MasterInterruptHandler(&imu);
}

