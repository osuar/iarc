#include <ros/ros.h>
#include "rxtxserver/Byte.h"
#include "rxtxserver/Read.h"
#include <stdlib.h>
#include <stdio.h>
#include <sensor_msgs/Joy.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>

class joystick
{
	public:
		joystick();
		int axes[5];
		int button[11];
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh_;
		ros:: Subscriber joy_sub_;
};

joystick::joystick(){
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &joystick::joyCallback, this);
}

void joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	int i;
	for(i = 0; i < 11; i++){
		button[i] = joy->buttons[i];
		//printf("%i", button[i]);
	}
	//printf("\n");
	for(i = 0; i < 5; i++){
		axes[i] = int(256*joy->axes[i]);
		//printf("%i", axes[i]);
	}
	//printf("\n");
}
	

void sendstring(char * string, ros::ServiceClient * client){
	int i = 0;

	rxtxserver::Byte srv;
	for(i = 0; i < 10; i ++){
		srv.request.outData = string[i];
		//srv.request.outData = ' ';

		(*client).call(srv);
		//printf("%c\n", srv.request.outData);
	}
}
		

int sv[2];
int main(int argc, char **argv){

	//joystick stick;
	socketpair(AF_UNIX, SOCK_STREAM, 0, sv);


	if(fork()){
		printf("Read Thread Alive\n");
		ros::init(argc, argv, "read_byte_client");

		ros::NodeHandle r;
		ros::ServiceClient rclient = r.serviceClient<rxtxserver::Read>("read_byte");
		rxtxserver::Read data;

		FILE * intercomm = fopen("intercomm", "w");

		while(1){
			printf("Read Client Attempting Read");
			if(rclient.call(data)){
				fputc(data.response.outData, intercomm);
				fflush(intercomm);
			}
			usleep(1000);
		}
		return 1;

	}
	else{
		printf("Write Thread Alive\n");

		ros::init(argc, argv, "send_byte_client");
		ros::NodeHandle n;
		ros::ServiceClient client = n.serviceClient<rxtxserver::Byte>("send_byte");

		FILE * intercomm2 = fopen("intercomm", "r");


		time_t start,now;


		char string[11];
		string[0] = ' ';
		string[1] = '0';
		string[2] = 'a';
		string[9] = 's';
		string[10] = 0;

		//int i;
		char buf;



		while(1){
			/*
			   ros::spinOnce();
			   for(i = 3; i < 8; i ++){
			   string[i] = char(stick.axes[i-3]);
			   }
			   string[8] = 20;
			   for(i = 0; i < 11; i ++){
			   if(stick.button[i]){
			   string[8] = i;
			   }
			   }
			   if(string[8] == 0){
			   string[8] = 20;
			   }
			 */
			/*
			   sendstring(string);

			   for(i = 0;i<10;i ++){
			   usleep(4000);

			   if(rclient.call(data)){
			   printf("%c", data.response.outData);
			   }

			   }
			 */
			printf("Write Thread Sending Packet\n");
			sendstring(string, &client);
			time(&start);
			do{
				usleep(1000);
				printf("checking data\n");
				if(fscanf(intercomm2, "%c", buf) == 1){
					printf("%c\n", buf);
				}
				time(&now);
			}while(difftime(now, start) < 0.04);
		}	
		printf("I'm Stopping for some reason\n");
		return 1;
	}
}
