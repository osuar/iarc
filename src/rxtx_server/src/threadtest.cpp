#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "rxtxserver/distance.h"
#include "rxtxserver/AngularPos.h"
#include "rxtxserver/status.h"

class status{
	public:
		status();
		char platStatus;
	private: void statusCallback(const rxtxserver::status::ConstPtr& status);
		ros::NodeHandle n;
		ros:: Subscriber status_sub;
};
status::status(){
	status_sub = n.subscribe<rxtxserver::status>("platStatus", 1, &status::statusCallback, this);
}
void status::statusCallback(const rxtxserver::status::ConstPtr& status){
	if(status->status){
		platStatus = status->status;
	}
}


class joystick
{
	public:
		joystick();
		int axes[5];
		int button[11]; 
	private: void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh_;
		ros:: Subscriber joy_sub_; }; 
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
		axes[i] = char(127*joy->axes[i]);
		//printf("%i", axes[i]);
	}
	//printf("\n");
}

class distance{
	public:
		distance();
		int vertical;
		char updateFlag;
	private: void lidarCallback(const rxtxserver::distance::ConstPtr& lidardata);
		ros::NodeHandle n;
		ros:: Subscriber distance_sub;
};
distance::distance(){
	distance_sub = n.subscribe<rxtxserver::distance>("lidar", 1, &distance::lidarCallback, this);
}
		
void distance::lidarCallback(const rxtxserver::distance::ConstPtr& lidardata){
	//ROS_INFO("Lidar is %d\n", lidardata->vertical);
	updateFlag = 1;
	vertical = lidardata->vertical;
}



void sendstring(char * string, FILE * serial){
	int i;
	for(i = 0; i < 10; i ++){
		fputc(string[i], serial);
	}
	fflush(serial);
}
		




int sv[2];


int main(int argc, char **argv){
	socketpair(AF_UNIX, SOCK_STREAM, 0, sv);

/*For Reading Serial in a nonblocking manner*/
	if(fork()){
		printf("Im thread 1!\n");
		ros::init(argc, argv, "rx");

		ros::NodeHandle n;

		ros::Publisher angular_pos = n.advertise<rxtxserver::AngularPos>("angular_pos", 5);
		ros::Publisher lidar = n.advertise<rxtxserver::distance>("lidar", 5);

		ros::Publisher status = n.advertise<rxtxserver::status>("platStatus", 1);
		rxtxserver::AngularPos angularposdata;
		rxtxserver::distance lidardata;
		rxtxserver::status platStatus;



		char buf[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};  

		char readcount = 0;
		//FILE * intercomm;
		FILE * serial;
		serial = fopen("/dev/ttyUSB1", "r");
		//intercomm = fopen("intercomm", "w");

		


		ros::spinOnce();	
		while(1){
			buf[readcount] = fgetc(serial);
			if((buf[0] == 'v') && (readcount == 0)){
				readcount ++;
				//printf("GOT A V\n");
			}
			else if((buf[1] == 'e') && (readcount == 1)){
				readcount ++;
				//printf("GOT A E\n");
			}
			else if((buf[2] == 'x') && (readcount == 2)){
				readcount ++;
				//printf("GOT A X\n");
			}
			else if((readcount >= 3) && (readcount <= 9)){
				readcount ++;
			}
			else if((buf[10] == 'u') && (readcount == 10)){

				//printf("GOT ONE\n");
				lidardata.vertical = (buf[3] << 8) | buf[4];
				lidar.publish(lidardata);
				platStatus.status = buf[9];
				status.publish(platStatus);
				readcount = 0;
			}
			else if((buf[readcount] == 'v')){
				readcount = 1;
			}
			else if((buf[readcount] == 'e')){
				readcount = 2;
			}
			else if((buf[readcount] == 'x')){
				readcount = 3;
			}
			//fputc(buf, intercomm);
			//fflush(intercomm);
/*
			if(stick.button[3]){
					printf("quitting\n");
					fclose(serial);
					fclose(intercomm);
					return 0;
			}
*/

		}

	}

/*Primary Thread, transmits packets and handles intelligence*/
	else{
		printf("Im thread 2!\n");
		ros::init(argc, argv, "rxtx");
		//ros::NodeHandle n;
		//ros::Subscriber lidar = n.subscribe("lidar", 5, lidarCallback);

		joystick stick;
		distance lidar;
		status platStatus;

		char myStatus = 1;

		lidar.updateFlag = 0;

		platStatus.platStatus = 1;

		FILE * intercomm;
		FILE * serial;
		intercomm = fopen("intercomm", "r");
		serial = fopen("/dev/ttyUSB1", "w");
		char buf[50];
		char serbuf[10];

		char prev = 20;

		int i;

		serbuf[0] = ' ';
		serbuf[1] = '0';
		serbuf[2] = 'a';
		serbuf[9] = 's';

		for(i=3;i<8;i++){
			serbuf[i] = 1;
		}

		serbuf[8] = 20;


		ros::spinOnce();
		for(i=0;i<11;i++){
			stick.button[i] = 0;
		}
		for(i=0;i<5;i++){
			stick.axes[i] = -126;
		}


		printf("I've made it this far\n");
		while(1){
			ros::spinOnce();

			if((platStatus.platStatus != myStatus) && (platStatus.platStatus != 20)){
				myStatus = platStatus.platStatus;
				if(myStatus == 1){
					printf("booting\n");
				}
				else if(myStatus == 2){
					printf("stopping\n");
				}
				else if(myStatus == 3){
					printf("running\n");
				}
				else if(myStatus == 4){
					printf("offsetting\n");
				}
			}
				
				

			if(stick.button[3]){
					printf("Thread 2 quitting\n");
					fclose(serial);
					fclose(intercomm);
					return 0;
			}

			if(lidar.updateFlag){
				lidar.updateFlag = 0;
				//printf("%i\n", lidar.vertical);
			}

			for(i = 3; i < 8; i ++){
				serbuf[i] = char(stick.axes[i-3]) + 126;
			}

			serbuf[8] = 20;
			for(i = 0; i < 11; i ++){
				if(stick.button[i]){
					serbuf[8] = i;
				}
			}
			if(serbuf[8] == prev){
				serbuf[8] = 20;
			}
			else{
				prev = serbuf[8];
			}

			//printf("%i\n", serbuf[8]);

		//	sendstring(serbuf, serial);
			for(i = 0; i < 10; i ++){
				fputc(serbuf[i], serial);
			}
			fflush(serial);

/*
			if(fscanf(intercomm, "%s", buf) >= 1){
				printf("%s\n", buf);
			}
*/
			

			usleep(40000);


		}


	}
	return 0;
}
