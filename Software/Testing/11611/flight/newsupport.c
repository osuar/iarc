/*
#include "../../drivers/avr_compiler.h"
#include "../../drivers/usart_driver.h"
#include "../../drivers/twi_master_driver.h"
#include "support.h"
*/
#include <stdio.h>
#include "newsupport.h"

#define ONE 62


int abs(int i){
	if(i < 0) {
		i *= -1;
		return i;
	}
	else {
		return i;
	}
}

int mysqrt(int input) {
	int i = 1;
	int root = 0;

	// If input is negative, don't do anything.
	if (input < 0) {
		return NULL;   // TODO: Light up an LED or something.
	}


	while (abs(input/i - i) > 1) {
		i = (i + input/i) / 2;
	}

	// Check which of i and i-1 is closer to the actual
	// root by comparing the absolute differences between
	// input and i^2 and between input and (i-1)^2.
	if(abs(input - i*i) > abs(input - (i-1)*(i-1))) {
		root = i-1;
	}
	else{
		root = i;
	}

	printf("%d\n", root);


	// Warn in we need to check for more possible roots (i.e., the upper
	// limit to 'i' in the for loop above should be increased or if the
	// input is unreasonably large).
	if (input - root*root < 0) {
		printf("Input number is too large!");   // TODO: Light up an LED or something.
		return NULL;
	}
	else {
		return i;
	}
}

void normalize(int * input,int * output){
	int i;
	int magnitude = 0;

	// Sum the squares of the three input items and take square root.
	for(i = 0; i < 3; i ++){
		magnitude += input[i] * input[i];
	}
	magnitude = mysqrt(magnitude);

	// Scale output by ONE/magnitude.
	for(i = 0; i < 3; i ++){
		output[i] = input[i] * ONE / magnitude;
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

void vectorScale(int * input, int * scale, int * output){
	int i;
	for(i = 0; i < 3; i ++){
		output[i] = input[i] * (*scale) / ONE;
	}
}

void orthoNormalize(int * matrix){
	int delta[6];
	int i;
	int error = -1/2 * dot(&matrix[0], &matrix[3]);
	vectorScale(&matrix[3], &error, &delta[0]);
	vectorScale(&matrix[0], &error, &delta[3]);
	vectorAdd(&matrix[0], &delta[0], &matrix[0]);
	vectorAdd(&matrix[3], &delta[3], &matrix[3]);

	cross(&matrix[0], &matrix[3], &matrix[6]);

	for(i = 0; i < 3; i ++){
		normalize(&matrix[i * 3], &matrix[i * 3]);
	}
}


void updateMatrix(int * accel, int * gyro, int* matrix){
	int i;
	int normaccel[3];
	int wAccel[3];
	int orient[3];
	int output[9];

	normalize(accel, normaccel);

	cross(&matrix[6], normaccel, wAccel);
	
	for(i = 0; i < 3; i ++){
		orient[i] = ((gyro[i] * GYROSCALE) + (wAccel[i] * ACCELSCALE))/(DIVFACTOR);
	}

	for(i = 0; i < 3; i ++){
		cross(&matrix[3 * i], orient, &output[i * 3]);
		vectorAdd(&matrix[3 * i], &output[i * 3], &matrix[3 * i]);
	}

	orthoNormalize(matrix);
}

void updateMotor(int * dcmMatrix, int * targetMatrix, int * rollint, int * motorSpeed){
	motorSpeed[0] = (MOTORONEGYRO * rollint[1]) + (MOTORONESCALE * dot(&targetMatrix[6], &dcmMatrix[3])) + MOTORONEINIT;
	motorSpeed[2] = (MOTORTHREEGYRO * rollint[1]) + (MOTORTHREESCALE * dot(&targetMatrix[6], &dcmMatrix[3])) + MOTORTHREEINIT;
	motorSpeed[1] = (MOTORTWOGYRO * rollint[0]) + (MOTORTWOSCALE * dot(&targetMatrix[6], &dcmMatrix[0])) + MOTORTWOINIT;
	motorSpeed[3] = (MOTORFOURGYRO * rollint[0]) + (MOTORFOURSCALE * dot(&targetMatrix[6], &dcmMatrix[0])) + MOTORFOURINIT;
}


