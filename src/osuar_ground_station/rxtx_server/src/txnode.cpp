#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <ncurses.h>
#include "ros/ros.h"
#include "rxtx_server/command.h"

class command{
	public:
		command();
		char throttleBig;
		char throttleLittle;
		char yaw; //rotation
		char pitch; //Fwd
		char roll; // side
		char button;
		char update;
	private: 
		void commandCallback(const rxtx_server::command::ConstPtr& command);
		ros::NodeHandle n;
		ros::Subscriber command_sub;
};
command::command(){
	command_sub = n.subscribe<rxtx_server::command>("command", 1, &command::commandCallback, this);
}
void command::commandCallback(const rxtx_server::command::ConstPtr& command){
	throttleBig = command->throttleBig;
	throttleLittle = command->throttleLittle;
	yaw = command->yaw;
	pitch = command->pitch;
	roll = command->roll;
	button = command->button;
	update = 1;
}

int main(int argc, char ** argv){
	initscr();
	cbreak();
	noecho();
	timeout(0);

	ros::init(argc, argv, "tx_node");

	char serbuf[10];

	int i;

	serbuf[0] = ' ';
	serbuf[1] = '0';
	serbuf[2] = 'a';
	serbuf[9] = 's';

	command comm;
	comm.button = 20;

	ros::spinOnce();

	FILE * serial;
	serial = fopen("/dev/ttyUSB0", "w");

	char keyboard;

	printw("TRANSMIT NODE HERE!\n");
	while(1){
		while(comm.update != 1){
			ros::spinOnce();
			if((keyboard = getch()) != ERR){
				if(keyboard == 's'){
					endwin();
					fclose(serial);
					return 0;
				}
			}

			usleep(200);
		}

		comm.update = 0;

		serbuf[3] = comm.roll;
		serbuf[4] = comm.pitch;
		serbuf[5] = comm.throttleLittle;
		serbuf[6] = comm.yaw;
		serbuf[7] = comm.throttleBig;
		serbuf[8] = comm.button;

		for(i = 0; i < 10; i ++){
			fputc(serbuf[i], serial);
		}
		fflush(serial);
	}
}


	
