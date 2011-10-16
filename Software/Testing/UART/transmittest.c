/**

@author Daniel Sidlauskas Miller

Xbee transmit test
*/

#include <stdio.h>
#include <stdlib.h>

 
int main(void)
{
	FILE *xbee = fopen("/dev/ttyUSB1", "r+");
	while(1)
	{
		fprintf(xbee, "Testing\n");
		sleep(1);
	}

	fclose(xbee);

	return 0;
}
