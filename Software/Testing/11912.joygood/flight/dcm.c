#include <stdlib.h>
#include "../../drivers/avr_compiler.h"
#include "../../drivers/usart_driver.h"
#include "../../drivers/twi_master_driver.h"
#include "support.h"
#include <stdio.h>
#include "dcm.h"

#define ONE 90
int abs(int i){
	if(i < 0) {
		i *= -1;
		return i;
	}
	else {
		return i;
	}
}

/* Code taken from Craig McQueen*/
uint32_t mySqrt(uint32_t input){
	uint32_t op  = input;

	uint32_t res = 0;
	uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type


	// "one" starts at the highest power of four <= than the argument.
	while (one > op)
	{
		one >>= 2;
	}

	while (one != 0)
	{
		if (op >= res + one)
		{
			op = op - (res + one);
			res = res +  2 * one;
		}
		res >>= 1;
		one >>= 2;
	}

	/* Do arithmetic rounding to nearest integer */
	if (op > res)
	{
		res++;
	}

	return res;
}

void mNormalize(int * output, int * input){
	char negFlag[9];
	int i;
	int j;
	for(i = 0; i < 3; i ++){
		normalize(&input[i * 3], &output[i * 3], &negFlag[i * 3]);
		for(j = 0; j < 3; j ++){
			if(negFlag[(i * 3) + j]){
				output[(i * 3) + j] *= -1;
			}
		}
	}
}
		 

void normalize(int * input,int * output, char * negFlag){
	int i;
	uint32_t magnitude = 0;
	int buffer[3] = {0,0,0};

	// Sum the squares of the three input items and take square root.
	for(i = 0; i < 3; i ++){
		
		if(input[i] < 0){
			negFlag[i] = 1;
		}
		else{
			negFlag[i] = 0;
		}
		
		magnitude += input[i] * input[i];
	}
	magnitude = mySqrt(magnitude);
	if(magnitude == 0){
		magnitude = 1;
	}

	// Scale output by ONE/magnitude.
	for(i = 0; i < 3; i ++){
		buffer[i] = ((abs(input[i]) * ONE) / magnitude);
		if(buffer[i] == 0){
			PORTF.OUT = 1;
		}
		
		output[i] = buffer[i];
		
			
		/*	
		if(negFlag[i]){
			output[i] = -output[i];
		}
		*/
	}
}
int dot(int * input1, int * input2){
	return ((input1[0] * input2[0]) + (input1[1] * input2[1]) + (input1[2] * input2[2])) / ONE;
}

void cross(int * input1, int * input2, int * output){
		output[0] = ((input1[1] * input2[2]) - (input1[2] * input2[1])) / ONE;
		output[1] = ((input1[2] * input2[0]) - (input1[0] * input2[2])) / ONE;
		output[2] = ((input1[0] * input2[1]) - (input1[1] * input2[0])) / ONE;
}

void mRotate(int * output, int * input){
	output[0] = input[0];
	output[1] = input[3];
	output[2] = input[6];
	output[3] = input[1];
	output[4] = input[4];
	output[5] = input[7];
	output[6] = input[2];
	output[7] = input[5];
	output[8] = input[8];
}

void mMultiply(int * outputMatrix, int * inputMatrix1, int * inputMatrix2){
	int i;
	int j;
	int bufferMatrix[9];
	int rotateMatrix[9]; 
	mRotate(rotateMatrix, inputMatrix1);


	for(j = 0; j < 3; j ++){
		for(i = 0; i < 3; i ++){
			bufferMatrix[i + (j * 3)] = dot(&rotateMatrix[i * 3], &inputMatrix2[j * 3]);
		}
	}
	for(i = 0; i < 9; i ++){
		outputMatrix[i] = bufferMatrix[i] * OMEGA;
	}
}

void mAdd(int * outputMatrix, int * inputMatrix1, int * inputMatrix2){
	int i;
	for(i = 0; i < 9; i ++){
		outputMatrix[i] = inputMatrix1[i] + inputMatrix2[i];
	}
}

void dcmInit(int * matrix){
	int i;
	for(i = 0; i < 9; i ++){
		matrix[i] = 0;
	}
	matrix[0] = ONE;
	matrix[4] = ONE;
	matrix[8] = ONE;
}

void vectorAdd(int * input1, int * input2, int * output){
	int i;
	for(i = 0; i < 3; i ++){
		output[i] = input1[i] + input2[i];
	}
}

void vectorScale(int * input, int scale, int * output){
	int i;
	for(i = 0; i < 3; i ++){
		output[i] = input[i] * (scale);
	}
}


void orthoNormalize(int * matrix){
	int delta[6];
	int error = -1/2 * dot(&matrix[0], &matrix[3]);
	vectorScale(&matrix[3], error, &delta[0]);
	vectorScale(&matrix[0], error, &delta[3]);
	vectorAdd(&matrix[0], &delta[0], &matrix[0]);
	vectorAdd(&matrix[3], &delta[3], &matrix[3]);

	cross(&matrix[0], &matrix[3], &matrix[6]);
	mNormalize(matrix, matrix);
}

void motorSpeed(int * pry,
		int * integration, 
		int * gyroint, 
		char * joystick, 
		int * motorSpeeds,
		int * pidValues,
		int * pidValuesDen){
	int i;
	for(i = 0; i < 4; i ++){
		motorSpeeds[i] = MOTORREG;
		//Joystick Throttle
		motorSpeeds[i] += joystick[2] * ZJOYSENS;
	}
	//For x axis rotating on Z
	motorSpeeds[0] -= (-(pry[1] - (joystick[0] * TILTJOYSENS)) * pidValues[0]/pidValuesDen[0]) + (integration[0] * pidValues[1]) + (gyroint[1] * pidValues[2]/pidValuesDen[2]);
	motorSpeeds[2] += (-(pry[1] - (joystick[0] * TILTJOYSENS)) * pidValues[0]/pidValuesDen[0]) + (integration[0] * pidValues[1]) + (gyroint[1] * pidValues[2]/pidValuesDen[2]);
	//Joystick Z axis rotate (slow down speed up both motors)
	motorSpeeds[0] += joystick[3] * ROTJOYSENS;
	motorSpeeds[2] += joystick[3] * ROTJOYSENS;
	
	//For y axis rotating on Z
	motorSpeeds[1] -= ((pry[0] - (joystick[1] * TILTJOYSENS)) * pidValues[0]/pidValuesDen[0]) + (integration[1] * pidValues[1]) - (gyroint[0] * pidValues[2]/pidValuesDen[2]);
	motorSpeeds[3] += ((pry[0] - (joystick[1] * TILTJOYSENS)) * pidValues[0]/pidValuesDen[0]) + (integration[1] * pidValues[1]) - (gyroint[0] * pidValues[2]/pidValuesDen[2]);
	//Joystick Z axis rotate (slow down speed up both motors)
	motorSpeeds[1] -= joystick[3] * ROTJOYSENS;
	motorSpeeds[3] -= joystick[3] * ROTJOYSENS;
}
