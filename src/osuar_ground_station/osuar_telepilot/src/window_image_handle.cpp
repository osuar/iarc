#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "osuar_telepilot/window_image_command.h"
#include "osuar_telepilot/window_image_request.h"
#include "window_image_handle.h"
#include <time.h>


class window_image_handle{
	public:
		window_image_handle();	
		//int integral;
		osuar_telepilot::window_image_command comm_out;
		
	private:
		void window_image_handleCallback(const osuar_telepilot::window_image_request::ConstPtr& window_image_follow_input);

		ros::NodeHandle n;
		ros::Subscriber command_sub;
		ros::Publisher command_pub; };
window_image_handle::window_image_handle(){
	command_pub = n.advertise<osuar_telepilot::window_image_command>("window_image_info", 1);
	command_sub = n.subscribe<osuar_telepilot::window_image_request>("window_image_request", 1, &window_image_handle::window_image_handleCallback, this);
}

void window_image_handle::window_image_handleCallback(const osuar_telepilot::window_image_request::ConstPtr& window_image_input){
	if(window_image_input->center < -SMALL_THRESHOLD){
		comm_out.pitch = 10;
	}
	else if(window_image_input->center < -BIG_THRESHOLD){
		comm_out.pitch = 15;
	}
	else if(window_image_input->center > SMALL_THRESHOLD){
		comm_out.pitch = -10;
	}
	else if(window_image_input->center > BIG_THRESHOLD){
		comm_out.pitch = -25;
	}
	command_pub.publish(comm_out);
}


int main(int argc, char ** argv){
	ros::init(argc, argv, "window_image");

	window_image_handle window_image_handler;

	ros::spin();
}
