#define OMEGA 2
#define CURRENTWEIGHT 2
int abs(int i);

/* Code taken from Craig McQueen*/
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
