#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <ros/ros.h>
#include "rxtxserver/Byte.h"
#include "rxtxserver/Read.h"
#include <stdio.h>
#include <stdlib.h>


int fd;
FILE * serial;


void open_port(void){
/*
	struct termios options;
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd==-1){
		perror("open_port: Unable to open /dev/ttyUSB0 - ");
	}
	else{
		fcntl(fd, F_SETFL, 0);
		tcgetattr(fd, &options);
		cfsetispeed(&options, 9600);
		cfsetospeed(&options, 9600);
		options.c_cflag |= (CLOCAL | CREAD);
		tcsetattr(fd, TCSANOW, &options);
		fcntl(fd, F_SETFL, 2048);


		printf("IM AWAKE!\n" );
	}
*/
	serial = fopen("/dev/ttyUSB0", "r+");	
	printf("IM AWAKE!\n");
	
}

bool send_byte(rxtxserver::Byte::Request& req,
		rxtxserver::Byte::Response& res){
/*
	int n = write(fd, &req.outData, 1);
	if(n == 1){
		return true;
	}
	else{
		return false;
	}
*/
	printf("Attempting Write\n");
	fputc(req.outData, serial);
	return true;

}

bool read_byte(rxtxserver::Read::Request& req,
		rxtxserver::Read::Response& res){
/*
	char buffer;
	int n = read(fd, &buffer, 1);
	if(n == 1){
		res.outData = buffer;
		printf("%c", buffer);
		return true;
	}
	else{
		//printf("Read Failed\n");
		return false;
	}
*/
	printf("Attempting Read\n");
	res.outData = fgetc(serial);
	return true;

}
		
int main(int argc, char **argv){
	open_port();
	ros::init(argc, argv, "rxtx_server");
	ros::NodeHandle n;
	ros::NodeHandle r;

	ros::ServiceServer writeservice = n.advertiseService("send_byte", send_byte);
	ros::ServiceServer readservice = r.advertiseService("read_byte", read_byte);
	ros::spin();
}
