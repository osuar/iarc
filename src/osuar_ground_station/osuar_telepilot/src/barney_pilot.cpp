#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <ncurses.h>
#include "barney_pilot.h"
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "rxtx_server/distance.h"
#include "rxtx_server/AngularPos.h"
#include "rxtx_server/status.h"
#include "rxtx_server/command.h" 
#include "osuar_telepilot/altitude_request.h"
#include "osuar_telepilot/altitude_command.h"
#include "osuar_telepilot/wall_command.h"
#include "osuar_telepilot/wall_request.h"
#include "osuar_telepilot/wall_command.h"
#include "osuar_telepilot/wall_request.h"
#include "rxtx_server/heading_pos.h"
#include "altitude.h"
#include "wall_follow.h"



int main(int argc, char ** argv){
	initscr();
	cbreak();
	noecho();
	timeout(0);
	
	ros::init(argc, argv, "Barney");

	status platStatus;
	joystick stick;
	distance lidar;
	angular_pos orientation;

	lidar.prev = 0;

	ros::NodeHandle n;
	ros::Publisher command = n.advertise<rxtx_server::command>("command", 1);
	rxtx_server::command commandData;

	ros::Publisher altitude_handler_pub = n.advertise<osuar_telepilot::altitude_request>("altitude_request", 1);
	osuar_telepilot::altitude_request altitude_request_data;
	altitude_request_data.p = 60;
	altitude_request_data.d = 0;
	altitude_request_data.i = 10;
	altitude_request_data.target = 50;
	altitude_request_data.status = STARTING;


	altitude_handler altitude_controller_data;

	ros::Publisher wall_handler_pub = n.advertise<osuar_telepilot::wall_request>("wall_request", 1);
	osuar_telepilot::wall_request wall_request_data;
	wall_request_data.p = 4;
	wall_request_data.i = 0;
	wall_request_data.d = 0;
	wall_request_data.target = 250;
	wall_request_data.status = FOLLOW;

	wall_handler wall_controller_data;
	
		char myStatus = 1;

		lidar.updateFlag = 0;
		lidar.prev = 0;

		platStatus.platStatus = 1;

		char prev = 20;

		int i;

		int throttle_little_buffer = 0;
		int throttle_big_buffer = 0;

		int roll_buffer = 0;

		ros::spinOnce();
		for(i=0;i<11;i++){
			stick.button[i] = 0;
		}
		for(i=0;i<5;i++){
			stick.axes[i] = -126;
		}


	char keyboard;
	char landingflag = 0;
	//char sent_status = 9;

	//printw("BARNEY TIME!\n");
	mvprintw(0,0,"STARTING   \n");
	while(1){
		ros::spinOnce();
		if((keyboard = getch()) != ERR){
			if(keyboard == 's'){
				endwin();
				return 0;
			}
			else if(keyboard == 'y'){
				altitude_request_data.p += 1;
			}
			else if(keyboard == 'h'){
				altitude_request_data.p -= 1;
			}
			else if(keyboard == 'u'){
				altitude_request_data.d += 1;
			}
			else if(keyboard == 'j'){
				altitude_request_data.d -= 1;
			}
			else if(keyboard == 'i'){
				altitude_request_data.i += 1;
			}
			else if(keyboard == 'k'){
				altitude_request_data.i -= 1;
			}

			//For starting takeoff
			else if((keyboard == 't') && ((altitude_request_data.status == STARTING) || (altitude_request_data.status == 7)) ){
				altitude_request_data.status = TAKEOFF;
				mvprintw(0,0,"TAKEOFF    \n");
			}
			else if((keyboard == 'l')){
				//altitude_request_data.status = LAND;
				landingflag = 1;
				altitude_request_data.target = 0;
				mvprintw(0,0,"LAND       \n");
			}
			else if(keyboard == 'r'){
				altitude_request_data.status = STARTING;
				mvprintw(0,0,"STARTING   \n");
			}
			
			mvprintw(15,40,"ALTITUDE:\n");
			mvprintw(16,40,"P: %4f\n",altitude_request_data.p * .2);
			mvprintw(17,40,"D: %4f\n",altitude_request_data.d * .2);
			mvprintw(18,40,"I: %4i/5625\n",altitude_request_data.i);



		}

		//TAKEOFF AND LAND AUTOMATIC STATUS CHANGES
		//When Taking Off, check to make transition to hover
		if((altitude_request_data.status == TAKEOFF) && (altitude_controller_data.status == HOVER)){
			altitude_request_data.status = HOVER;
			mvprintw(0,0,"HOVERING      \n");
		}
		else if((altitude_request_data.status == HOVER) && (altitude_controller_data.status == LAND) && (landingflag == 0)){
			altitude_request_data.status = TAKEOFF;
			mvprintw(0,0,"TAKEOFF    \n");

		}
		else if((landingflag == 1) && (altitude_controller_data.status == LAND)){
			landingflag = 0;
			altitude_request_data.status = LAND;
		}
		
		if(orientation.updateFlag){
			orientation.updateFlag = 0;
			mvprintw(2,0,"X: %4i\n",orientation.x);
			mvprintw(3,0,"Y: %4i\n",orientation.y);
			mvprintw(4,0,"Z: %4i\n",orientation.z);
		}

			myStatus = platStatus.platStatus;
			if(myStatus == 9){
				mvprintw(1,0,"booting  \n");

				//Don't have bad things happen when module resets
				//sent_status = 9;
			}
			else if(myStatus == 4){
				mvprintw(1,0,"stopping \n");
				altitude_request_data.status = 7;
				altitude_controller_data.throttle_big = 0;
				altitude_controller_data.throttle_little = 0;
			}
			else if(myStatus == 1){
				mvprintw(1,0,"running  \n");
			}
			else if(myStatus == 10){
				mvprintw(1,0,"offsetting\n"); 
			}

		if(lidar.updateFlag){
			lidar.updateFlag = 0;
			mvprintw(5,0,"Vertical: %4i\n", lidar.vertical);
			mvprintw(6,0,"Horizontal 1: %4i\n", lidar.horizontal_1);
			if((abs(orientation.x) < 200) && (abs(orientation.y) < 200)){
				altitude_request_data.distance = lidar.vertical;
				altitude_handler_pub.publish(altitude_request_data);
				wall_request_data.distance = lidar.horizontal_1;
				wall_handler_pub.publish(wall_request_data);
			}
			else{
				//altitude_controller_data.throttle_big = 0;
				//altitude_controller_data.throttle_little = 0;

			}
		}
		throttle_little_buffer = stick.axes[2] + altitude_controller_data.throttle_little;
		throttle_big_buffer = stick.axes[4] + altitude_controller_data.throttle_big;

		//Will eventually be phased out of this section of code and put in altitude
/*
		if(throttle_big_buffer > 127){
			throttle_big_buffer = 127;
		}
		else if(throttle_big_buffer < -127){
			throttle_big_buffer = -127;
		}
		if(throttle_little_buffer > 127){
			if(throttle_big_buffer < 94){
				throttle_little_buffer -= 127;
				throttle_big_buffer += 32;
			}
			else{
				throttle_little_buffer = 127;
			}
		}
		else if(throttle_little_buffer < -127){
			if(throttle_big_buffer > -94){
				throttle_little_buffer += 127;
				throttle_big_buffer -= 32;
			}
			else{
				throttle_little_buffer = -127;
			}
		}
*/
/*
		throttle_overflow = throttle_little_buffer/127;
		throttle_little_buffer %= 127;
		throttle_big_buffer += throttle_overflow * 32;
*/	
		while(throttle_little_buffer > 127){
			throttle_little_buffer -= 127;
			throttle_big_buffer += 32;
		}
		while(throttle_little_buffer < -127){
			throttle_little_buffer += 127;
			throttle_big_buffer -= 32;
		}

		if(throttle_big_buffer > 127){
			throttle_big_buffer = 127;
		}
		else if(throttle_big_buffer < -127){
			throttle_big_buffer = -127;
			throttle_little_buffer = -127;
		}
		roll_buffer = stick.axes[0];//+ wall_controller_data.tilt;
		if(roll_buffer > 127){
			roll_buffer = 127;
		}
		else if(roll_buffer < -127){
			roll_buffer = -127;
		}

		commandData.roll = char(roll_buffer) + 127;
		commandData.pitch = char(stick.axes[1]) + 127;
		commandData.throttleLittle = char(throttle_little_buffer) + 127;
		commandData.yaw = char(stick.axes[3]) + 127;
		commandData.throttleBig = char(throttle_big_buffer) + 127;

		commandData.button = 20;
		for(i = 0; i < 11; i ++){
			if(stick.button[i]){
				commandData.button = i;
			}
		}

		//For status alignment varification
/*
		if(commandData.button == 9){
			sent_status = 9;
		}
		else if(commandData.button == 4){
			sent_status = 4;
		}
		else if(commandData.button == 1){
			sent_status = 1;
		}
		else if(commandData.button == 10){
			sent_status = 10;
		}
		else if(sent_status != myStatus){
			commandData.button = sent_status;
		}
*/
		if(commandData.button == prev){
			commandData.button = 20;
		}
		else{
			prev = commandData.button;
		}


		mvprintw(10,0,"pitch: %4d\n", commandData.pitch);
		mvprintw(11,0,"roll: %4d\n", commandData.roll);
		mvprintw(12,0,"yaw: %4d\n", commandData.yaw);
		mvprintw(13,0,"throttleLittle: %4d\n", commandData.throttleLittle);
		mvprintw(14,0,"throttleBig: %4d\n", commandData.throttleBig);
		mvprintw(15,0,"throttleLittle Buffer: %4d", stick.axes[2]);
		mvprintw(16,0,"throttleBig Buffer: %4d", stick.axes[4]);

		command.publish(commandData);

		usleep(16600);


	}
}
	
