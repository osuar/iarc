#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "osuar_telepilot/altitude_command.h"
#include "osuar_telepilot/altitude_request.h"
#include "altitude.h"
#include <time.h>

class altitude_handle{
	public:
		altitude_handle();	
		int read_number;
		int target;
		int distance_cache[5];//for differential term
		int integral;
		osuar_telepilot::altitude_command comm_out;
		
	private:
		void altitude_handleCallback(const osuar_telepilot::altitude_request::ConstPtr& altitude_input);

		ros::NodeHandle n;
		ros::Subscriber command_sub;
		ros::Publisher command_pub;
};
altitude_handle::altitude_handle(){
	command_pub = n.advertise<osuar_telepilot::altitude_command>("altitude_info", 1);
	command_sub = n.subscribe<osuar_telepilot::altitude_request>("altitude_request", 1, &altitude_handle::altitude_handleCallback, this);
}

void altitude_handle::altitude_handleCallback(const osuar_telepilot::altitude_request::ConstPtr& altitude_input){
	
	int i;
	int differential = 0;
	int position = 0;;
	int throttle_buffer;
	int throttle_big_buffer;
	int rate_position;
	int rate_target;
	target = altitude_input->target;
	if(altitude_input->status == STARTING){
		integral = 0;
		comm_out.throttle_little = 0;
		comm_out.throttle_big = 0;
		comm_out.status = LAND;

		read_number = 0;
		for(i=0;i<5;i++){
			distance_cache[i] = 0;
		}
		command_pub.publish(comm_out);

	}
	if(altitude_input->status == TAKEOFF){
		comm_out.status = HOVER;
		command_pub.publish(comm_out);
	}
	else if(altitude_input->status == HOVER){
		//If close to ground, don't modify throttle
		if((altitude_input->distance > 200) || (altitude_input->distance < 16)){
			comm_out.status = LAND; 
		}
		//Otherwise, should make constance, but setting basic height to 80 (~4ft)
		else{
			comm_out.status = HOVER; 
		}
		if(read_number < 5){
			distance_cache[read_number] = altitude_input->distance;
			read_number ++;
			differential = 0;
		}
		else{

			for(i=0;i<4;i++){
				distance_cache[i] = distance_cache[i+1];
			}
			distance_cache[4] = altitude_input->distance;
			differential = 0;
			differential -= distance_cache[4] - distance_cache[3];
			differential -= distance_cache[3] - distance_cache[2];
			differential -= distance_cache[2] - distance_cache[1];
			differential -= distance_cache[1] - distance_cache[0];
			differential *= -1;
			//differential = differential * altitude_input->d/5;
		}

		position = target - distance_cache[4];
		integral += position;
		//position = (position * altitude_input->p /5);

		if(abs(position) < 5){
			rate_target = 0;
		}
		else if(position > 0){
			if(position < 10){
				rate_target = 1;
			}
			else if(position < 15){
				rate_target = 2;
			}
			else if(position < 20){
				rate_target = 3;
			}
			else{
				rate_target = 4;
			}
		}
		else{
			if(position > -10){
				rate_target = -1;
			}
			else if(position > -15){
				rate_target = -2;
			}
			else if(position > -20){
				rate_target = -3;
			}
			else if(position > -30){
				rate_target = -4;
			}
			else{
				rate_target = -6;
			}
		}
		rate_position = rate_target - differential;	

		rate_position = (rate_position * altitude_input->p)/5;
		throttle_buffer = rate_position;// + (differential));
		throttle_big_buffer = integral * altitude_input->i / 5625;
		if(throttle_big_buffer > 32){
			throttle_big_buffer = 32;
		}
		else if(throttle_big_buffer < -32){
			throttle_big_buffer = -32;
		}
		if(throttle_buffer > THROTTLE_MAX){
			throttle_buffer = THROTTLE_MAX;
		}
		else if(throttle_buffer < -THROTTLE_MAX){
			throttle_buffer = -THROTTLE_MAX;
		}

		comm_out.throttle_little = throttle_buffer;
		comm_out.throttle_big = throttle_big_buffer + HOVER_SPEED;
		//printf("%i\n",comm_out.throttle_big);
		command_pub.publish(comm_out);
	}
	else if(altitude_input->status == LAND){
		comm_out.throttle_little = -127;
		comm_out.throttle_big = LAND_SPEED;
		command_pub.publish(comm_out);

	}
	
}		
		




int main(int argc, char ** argv){
	ros::init(argc, argv, "altitude");

	altitude_handle altitude_handler;

	altitude_handler.target = 60;
	altitude_handler.read_number = 0;
	altitude_handler.integral = 0;
	int i;
	for(i=0;i<5;i++){
		altitude_handler.distance_cache[i] = 0;
	}

	ros::spin();
}
