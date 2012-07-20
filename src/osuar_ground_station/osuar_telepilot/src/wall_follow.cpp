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
		int integral;
		osuar_telepilot::wall_command comm_out;
		
	private:
		void wall_handleCallback(const osuar_telepilot::wall_request::ConstPtr& wall_follow_input);

		ros::NodeHandle n;
		ros::Subscriber command_sub;
		ros::Publisher command_pub;
};
wall_handle::wall_handle(){
	command_pub = n.advertise<osuar_telepilot::wall_command>("wall_info", 1);
	command_sub = n.subscribe<osuar_telepilot::wall_request>("wall_request", 1, &wall_handle::wall_handleCallback, this);
}

void wall_handle::wall_handleCallback(const osuar_telepilot::wall_request::ConstPtr& wall_input){
	int i;
	int differential = 0;
	int position = 0;;
	int tilt_buffer;
	int integral_out = 0;
	target = wall_input->target;

	if(wall_input->status == FOLLOW){
		if(wall_input->distance > 200){

		}
		else{
			if(read_number < 5){
				distance_cache[read_number] = wall_input->distance;
				read_number ++;
				differential = 0;
			}
			else{
				if(wall_input->distance != distance_cache[4]){

					for(i=0;i<4;i++){
						distance_cache[i] = distance_cache[i+1];
					}
					distance_cache[4] = wall_input->distance;
				}
				differential = 0;
				differential -= distance_cache[4] - distance_cache[3];
				differential = differential * wall_input->d/5;
			}

			position = target - distance_cache[4];
			integral += position;
			position = (position * wall_input->p /5);
			integral_out = (integral * wall_input->i/5);

		}
		if(integral_out > INTEGRAL_WALL_MAX){
			integral_out = INTEGRAL_WALL_MAX;
		}
		else if(integral_out < -INTEGRAL_WALL_MAX){
			integral_out = -INTEGRAL_WALL_MAX;
		}
		if(differential > DIF_WALL_MAX){
			differential = DIF_WALL_MAX;
		}
		else if(differential < -DIF_WALL_MAX){
			differential = -DIF_WALL_MAX;
		}
		if(position > POS_WALL_MAX){
			position = POS_WALL_MAX;
		}
		else if(position < -POS_WALL_MAX){
			position = -POS_WALL_MAX;
		}
		tilt_buffer = int((position) + (differential) + (integral_out));
		if(tilt_buffer > TILT_MAX){
			tilt_buffer = TILT_MAX;
		}
		else if(tilt_buffer < -TILT_MAX){
			tilt_buffer = -TILT_MAX;
		}

		comm_out.tilt = tilt_buffer;
		//printf("%i\n",comm_out.tilt);
		command_pub.publish(comm_out);
	}

}		
		




int main(int argc, char ** argv){
	ros::init(argc, argv, "wall");

	wall_handle wall_handler;

	wall_handler.target = 60;
	wall_handler.read_number = 0;
	wall_handler.integral = 0;
	int i;
	for(i=0;i<5;i++){
		wall_handler.distance_cache[i] = 0;
	}

	ros::spin();
}
