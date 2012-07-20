#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "osuar_telepilot/wall_command.h"
#include "osuar_telepilot/wall_request.h"
#include "wall_follow.h"
#include <time.h>


class wall_handle{
	public:
		wall_handle();	
		int read_number;
		int target;
		int distance_cache[5];//for differential term
		//int integral;
		osuar_telepilot::wall_command comm_out;
		
	private:
		void wall_handleCallback(const osuar_telepilot::wall_request::ConstPtr& wall_follow_input);

		ros::NodeHandle n;
		ros::Subscriber command_sub;
		ros::Publisher command_pub; };
wall_handle::wall_handle(){
	command_pub = n.advertise<osuar_telepilot::wall_command>("wall_info", 1);
	command_sub = n.subscribe<osuar_telepilot::wall_request>("wall_request", 1, &wall_handle::wall_handleCallback, this);
}

void wall_handle::wall_handleCallback(const osuar_telepilot::wall_request::ConstPtr& wall_input){
	int i;
	int differential = 0;
	int position = 0;;
	int tilt_buffer;
	int rate_position;
	int rate_target;
	target = wall_input->target;

	if(read_number < 5){
		distance_cache[read_number] = wall_input->distance;
		read_number ++;
		differential = 0;
	}
	else{

		for(i=0;i<4;i++){
			distance_cache[i] = distance_cache[i+1];
		}
		distance_cache[4] = wall_input->distance;
		differential = 0;
		differential += distance_cache[4] - distance_cache[3];
		differential += distance_cache[3] - distance_cache[2];
		differential += distance_cache[2] - distance_cache[1];
		differential += distance_cache[1] - distance_cache[0];
		differential *= -1;
	}

	position = target - distance_cache[4];
	//integral += position;

	if(abs(position) < 20){
		rate_target = 0;
	}
	else if(position > 0){
		if(position < 40){
			rate_target = 5;
		}
		else if(position < 60){
			rate_target = 8;
		}
		else if(position < 80){
			rate_target = 11;
		}
		else{
			rate_target = 15;
		}
	}
	else{
		if(position > -40){
			rate_target = -5;
		}
		else if(position > -60){
			rate_target = -8;
		}
		else if(position > -80){
			rate_target = -11;
		}
		else{
			rate_target = -15;
		}
	}
	rate_position = rate_target - differential;	

	rate_position = (rate_position * wall_input->p)/5;
	tilt_buffer = rate_position;

	if(tilt_buffer > TILT_MAX){
		tilt_buffer = TILT_MAX;
	}
	else if(tilt_buffer < -TILT_MAX){
		tilt_buffer = -TILT_MAX;
	}

	comm_out.tilt = tilt_buffer;
	command_pub.publish(comm_out);
}
	
		




int main(int argc, char ** argv){
	ros::init(argc, argv, "wall");

	wall_handle wall_handler;

	wall_handler.read_number = 0;
	//wall_handler.integral = 0;
	int i;
	for(i=0;i<5;i++){
		wall_handler.distance_cache[i] = 0;
	}

	ros::spin();
}
