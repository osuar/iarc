#include <stdio.h>
#include <stdlib.h>

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
		outputMatrix[i] = bufferMatrix[i];
	}
}
int dot(int * input1, int * input2){
	return ((input1[0] * input2[0]) + (input1[1] * input2[1]) + (input1[2] * input2[2]));
}
void printMatrix(int * matrix){
	int i;
	int j;
	for(i = 0; i < 3; i ++){
		printf("( ");
		for(j = 0; j < 3; j ++){
			printf("%d ", matrix[j + (i * 3)]);
		}
		printf(") ");
	}
	printf("\n");
}


int main(void){
	int matrix1[] = {1,0,0,0,1,0,0,0,1};
	int matrix2[] = {2,4,5,0,2,0,0,0,2};
	int rotateMatrix[9];
	int i;
	int j;
	mMultiply(matrix1, matrix1, matrix2);
	printMatrix(matrix1);

/*
	int i = 12;

//	printf("%d\n", mysqrt(i));

	int normtest[] = {10, 30, 80};
	int normalized[3];
	int crosstest[] = {0, 0, 6};
	int crossed[3];

	//int a[] = {1000, 234, 183};
	//int b[] = {0,0,0};

	//normalize(a, b);
	//printf("\n\n{%d, %d, %d}\n\n", b[0], b[1], b[2]);
	int j;
	for (j=0; j<20; j++) {
		printf("sqrt(%d) = ", j);
		mysqrt(j);
	}

//	printf("Hello!\n\r");
//	normalize(normtest, normalized);
//	printf("%d %d %d\n", normalized[0], normalized[1], normalized[2]);
//	printf("%d\n", dot(normtest, normalized));
//	cross(normtest, crosstest, crossed);
//	printf("%d %d %d\n", crossed[0], crossed[1], crossed[2]);
*/
	return 0;

}
