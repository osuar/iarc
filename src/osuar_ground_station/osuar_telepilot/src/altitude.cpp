#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <ncurses.h>
#include "ros/ros.h"
#include "osuar_telepilot/altitudeCommand.h"

class altitudeHandle{
	public:
		altitudeHandle();	
		int read_number;
		int distance_cache[5];//for differential term
		osuar_telepilot::altitude_output comm_out;
		
	private:
		void altitudeHandleCallback(const osuar_telepilot::altitude_input::ConstPtr& altitude_input);

		ros::NodeHandle n;
		ros::Subscriber command_sub;
		ros::Publisher command_pub;
};
altitudeHandle::altitudeHandle(){
	command_pub = n.advertise<osuar_telepilot::command>("alititude_info", 1);
	command_sub = n.subscribe<osuar_telepilot::status>("altitude_request", 1, &altitudeHandle::altitudeHandleCallback, this);
}

void altitudeHandle::altitudeHandleCallback(const osuar_telepilot::altitude_input::ConstPtr& altitude_input){
	int i;
	int differential;
	//If close to ground, don't modify throttle
	if(altitude_input->distance > 200){
		comm_out.throttle = 0;
		comm_out.status = 0; //landed (should make constant)
		read_number = 0;
		for(i=0;i<5;i++0){
			distance_cache[i] = 0;
		}
		command_pub.publish(comm_out);
	}
	//Otherwise, should make constance, but setting basic height to 80 (~4ft)
	else if(altitude_input->distance > 30){
		comm_out.status = 1; //hovering (should make constant)
		if(readnumber < 5){
			distance_cache[readnumber] = altitude_input->distance;
			readnumber ++;
		}
		else{
			for(i=0;i<4;i++){
				read_cache[i] = read_cache[i+1];
			}
			read_cache[4] = altitude_input->distance;
		}
		differential = read_cache[4] - read_cache[0];
		
		




int main(int argc, char ** argv){
	initscr();
	cbreak();
	noecho();
	timeout(0);
	
	ros::init(argc, argv, "altitude");

