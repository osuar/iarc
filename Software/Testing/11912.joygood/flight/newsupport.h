#define MOTORONEINIT 2000
#define MOTORTWOINIT 2000
#define MOTORTHREEINIT 2000
#define MOTORFOURINIT 2000


#define MOTORONESCALE -1
#define MOTORTWOSCALE 1
#define MOTORTHREESCALE 1
#define MOTORFOURSCALE 1

#define MOTORONEGYRO 1
#define MOTORTWOGYRO 1
#define MOTORTHREEGYRO 1
#define MOTORFOURGYRO 1

#define GYROSCALE 5
#define ACCELSCALE 1
#define DIVFACTOR 6

int mysqrt(int input);

void normalize(int * input,int * output);

int dot(int * input1, int * input2);

void cross(int * input1, int * input2, int * output);

void dcmInit(int * matrix);

void vectorAdd(int * input1, int * input2, int * output);

void vectorScale(int * input, int * scale, int * output);

void orthoNormalize(int * matrix);
	
void updateMatrix(int * accel, int * gyro, int* matrix);

void updateMotor(int * dcmMatrix, int * targetMatrix, int * rollint, int * motorSpeed);
