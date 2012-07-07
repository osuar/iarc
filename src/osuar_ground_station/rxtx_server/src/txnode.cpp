#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <ncurses.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "rxtx_server/distance.h"
#include "rxtx_server/AngularPos.h"
#include "rxtx_server/status.h"


class status{
	public:
		status();
		char platStatus;
	private: void statusCallback(const rxtx_server::status::ConstPtr& status);
		ros::NodeHandle n;
		ros:: Subscriber status_sub;
};
status::status(){
	status_sub = n.subscribe<rxtx_server::status>("platStatus", 1, &status::statusCallback, this);
}
void status::statusCallback(const rxtx_server::status::ConstPtr& status){
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
		//printw("%i", button[i]);
	}
	//printf("\n");
	for(i = 0; i < 5; i++){
		axes[i] = char(127*joy->axes[i]);
		//printw("%i", axes[i]);
	}
	//printw("\n");
}

class distance{
	public:
		distance();
		int vertical;
		char updateFlag;
	private: void lidarCallback(const rxtx_server::distance::ConstPtr& lidardata);
		ros::NodeHandle n;
		ros:: Subscriber distance_sub;
};
distance::distance(){
	distance_sub = n.subscribe<rxtx_server::distance>("lidar", 1, &distance::lidarCallback, this);
}
		
void distance::lidarCallback(const rxtx_server::distance::ConstPtr& lidardata){
	//ROS_INFO("Lidar is %d\n", lidardata->vertical);
	updateFlag = 1;
	vertical = lidardata->vertical;
}

class angular_pos{
	public:
		angular_pos();
		int x;
		int y;
		int z;
		char updateFlag;
	private:
		void angular_posCallback(const rxtx_server::AngularPos::ConstPtr& angularposdata);
		ros::NodeHandle n;
		ros::Subscriber angular_pos_sub;
};
angular_pos::angular_pos(){
	angular_pos_sub = n.subscribe<rxtx_server::AngularPos>("angular_pos", 1, &angular_pos::angular_posCallback, this);
}

void angular_pos::angular_posCallback(const rxtx_server::AngularPos::ConstPtr& angularposdata){
	updateFlag = 1;
	x = angularposdata->x;
	y = angularposdata->y;
	z = angularposdata->z;
}




int main(int argc, char ** argv){
	initscr();
	cbreak();
	noecho();
	timeout(0);
	
	ros::init(argc, argv, "primary");

	status platStatus;
	joystick stick;
	distance lidar;
	angular_pos orientation;


		char myStatus = 1;

		lidar.updateFlag = 0;

		platStatus.platStatus = 1;

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





	FILE * serial;
	serial = fopen("/dev/ttyUSB0", "w");

	char keyboard;

	printw("PRIMARY NODE HERE!\n");
	while(1){
		ros::spinOnce();
		if((keyboard = getch()) != ERR){
			if(keyboard == 's'){
				endwin();
				fclose(serial);
				return 0;
			}
		}
		
		if(orientation.updateFlag){
			orientation.updateFlag = 0;
			mvprintw(2,0,"X: %4i\n",orientation.x);
			mvprintw(3,0,"Y: %4i\n",orientation.y);
			mvprintw(4,0,"Z: %4i\n",orientation.z);
		}

		if((platStatus.platStatus != myStatus) && (platStatus.platStatus != 20)){
			myStatus = platStatus.platStatus;
			if(myStatus == 1){
				mvprintw(1,0,"booting  \n");
			}
			else if(myStatus == 2){
				mvprintw(1,0,"stopping \n");
			}
			else if(myStatus == 3){
				mvprintw(1,0,"running  \n");
			}
			else if(myStatus == 4){
				mvprintw(1,0,"offsetting\n");
			}
		}

		if(lidar.updateFlag){
			lidar.updateFlag = 0;
			mvprintw(5,0,"Vertical: %4i\n", lidar.vertical);
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

		for(i = 0; i < 10; i ++){
			fputc(serbuf[i], serial);
		}
		fflush(serial);

		usleep(40000);


	}
}

	
