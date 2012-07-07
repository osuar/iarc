#include <stdio.h>
#include <stdlib.h>
//#include <file.h>

int main(void){
	FILE * serial;

	serial = fopen("/dev/ttyUSB0", "r+");
	int i;

	
char c;
	for(i = 0; i < 20; i ++){
		while(feof(serial));
		c = fgetc(serial);
		printf("%c\n", c);
	
	
/*
	fputc('a', serial);
	fputc('\n', serial);
	sleep(1);
*/
	}

	fclose(serial);
}
