#include <stdio.h>
#include <stdlib.h>

int main(void){
	FILE * xbee;
	xbee = fopen("/dev/ttyUSB0", "w+");
	char xbeebuffer[7];
	xbeebuffer[0] = ' ';
	xbeebuffer[1] = ' ';
	
	
	while(1){
		xbeebuffer[2] = 100;
		xbeebuffer[3] = 57;
		xbeebuffer[4] = -25;
		xbeebuffer[5] = 77;
		xbeebuffer[6] = ' ';
		fprintf(xbee, "%s\n\r", xbeebuffer);
		sleep(1);
		
	}
}
		
		
