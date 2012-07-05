#include <stdlib.h>
#include <stdio.h>
#include <ncurses.h>
#include "ros/ros.h"
#include "rxtxserver/distance.h"
#include "rxtxserver/AngularPos.h"
#include "rxtxserver/status.h"


int main(int argc, char **argv){
	initscr();
	cbreak();
	noecho();
	timeout(0);

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



	FILE * serial;
	serial = fopen("/dev/ttyUSB0", "r");

	int i;
	for(i=0;i<14;i++){
		buf[i] = 0;
	}

	char keyboard;

	printw("IM HERE\n");
	while(1){
		buf[readcount] = fgetc(serial);
		if((buf[0] == 'v') && (readcount == 0)){
			readcount ++;
			//printw("GOT A V\n");
		}
		else if((buf[1] == 'e') && (readcount == 1)){
			readcount ++;
			//printw("GOT A E\n");
		}
		else if((buf[2] == 'x') && (readcount == 2)){
			readcount ++;
			//printw("GOT A X\n");
		}
		else if((readcount >= 3) && (readcount <= 9)){
			readcount ++;
		}
		else if((buf[10] == 'u') && (readcount == 10)){

			//printw("GOT ONE\n");
			lidardata.vertical = (buf[3] << 8) | buf[4];
			lidar.publish(lidardata);
			platStatus.status = buf[9];
			status.publish(platStatus);
			angularposdata.x = (buf[5] << 8) | buf[6];
			angularposdata.y = (buf[7] << 8) | buf[8];
			angular_pos.publish(angularposdata);
			
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

		if((keyboard = getch()) != ERR){
			if(keyboard == 's'){
				endwin();
				fclose(serial);
				return 0;
			}
		}
	}
}


