#include <stdio.h>
#include <stdlib.h>
#include "newsupport.h"

int main(void){

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
	return 0;

}
