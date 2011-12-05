#ifndef DCM_H
#define DCM_H

#define OMEGA 4 / 10
#define AVEC 1/3
#define AWEIGHT 3
#define CURRENTWEIGHT 4/3
#define AXISONZ 10
#define ROTONZ 3
#define ROTJOYSENS 1
#define ZJOYSENS 2
#define TILTJOYSENS 1
#define MOTORREG 2000

int abs(int i);

/* Code inspired from Craig McQueen's similar functions*/
void motorSpeed(int * dcmMatrix,int * gyroint, int * joystick, int * motorSpeeds);
uint32_t mySqrt(uint32_t input);
void normalize(int * input,int * output, char * negFlag);
void mNormalize(int * output, int * input);
int dot(int * input1, int * input2);
void dcmInit(int * matrix);
void vectorAdd(int * input1, int * input2, int * output);
void vectorScale(int * input, int scale, int * output);
void orthoNormalize(int * matrix);
void mAdd(int * outputMatrix, int * inputMatrix1, int * inputMatrix2);
void mMultiply(int * outputmatrix, int * inputMatrix1, int * inputMatrix2);

#endif // DCM_H
