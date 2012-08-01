#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "osuar_telepilot/opening_command.h"
#include "osuar_telepilot/opening_request.h"
#include "opening_handle.h"
#include <time.h>


class opening_handle{
	public:
		opening_handle();	
		//int integral;
		osuar_telepilot::opening_command comm_out;
		
	private:
		void opening_handleCallback(const osuar_telepilot::opening_request::ConstPtr& opening_follow_input);

		ros::NodeHandle n;
		ros::Subscriber command_sub;
		ros::Publisher command_pub; };
opening_handle::opening_handle(){
	command_pub = n.advertise<osuar_telepilot::opening_command>("opening_info", 1);
	command_sub = n.subscribe<osuar_telepilot::opening_request>("opening_request", 1, &opening_handle::opening_handleCallback, this);
}

void opening_handle::opening_handleCallback(const osuar_telepilot::opening_request::ConstPtr& opening_input){
	int leftflag = 0;
	int rightflag = 0;
	if(opening_input->left < 100){
		leftflag = 1;	
	}
	
	if(opening_input->right < 100){
		rightflag = 1;
	}

	if(rightflag && leftflag){
		comm_out.status = O_WALL;
	}
	else if(rightflag){
		comm_out.status = O_LEFT;
	}
	else if(leftflag){
		comm_out.status = O_RIGHT;
	}
	else{
		comm_out.status = O_FORWARD;
	}
	command_pub.publish(comm_out);
}


int main(int argc, char ** argv){
	ros::init(argc, argv, "opening");

	opening_handle opening_handler;

	ros::spin();
}
