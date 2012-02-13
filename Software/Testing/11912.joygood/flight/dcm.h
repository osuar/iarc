#define DEGREE 10

//Scales incoming gyro values to arbitrary DCM matrix size, subject to change
#define OMEGA 10 / 10

//Scales combination of accelerometer data and gyro data to estimated Accle
//ration vector, subject to change
#define AVEC 3/2

//Weight of accelerometer values in creation of acceleration vector
#define AWEIGHT 0

#define ACCSCALE 3/2

//Weight of current matrix in combination to make new matrix
#define CURRENTWEIGHT 1
#define DECAYWEIGHT 0
//#define AXISONZ 1/7
//#define ROTONZ 20
//#define INTONZ 3/10
#define INTCONST 15/16
#define ROTJOYSENS 1
#define ZJOYSENS 1
#define TILTJOYSENS 3/7
#define MOTORREG 3275

int abs(int i);

void cross(int * input1, int * input2, int * output);

void motorSpeed(int * pry,int * integration, int * gyroint, char * joystick, int * motorSpeeds, int * pidValues, int * pidValuesDen);

/* Sqrrt Code inspired by Craig McQueen's similar function*/
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
