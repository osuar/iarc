#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "rxtx_server/distance.h"
#include "rxtx_server/AngularPos.h"
#include "rxtx_server/status.h"
#include "rxtx_server/command.h"
#include "rxtx_server/heading_pos.h"
#include "osuar_telepilot/altitude_request.h"
#include "osuar_telepilot/altitude_command.h"
#include "osuar_telepilot/wall_command.h"
#include "osuar_telepilot/wall_request.h"
#include "altitude.h"

/*
Needs work, but might no be necessary
class heading_controller{
	public:
		heading_controller();
		char yaw;
	private:
		void heading_controllerCallback(const rxtx_server::heading_control::ConstPtr& heading);
		ros::NodeHandle n;
		ros::Subscriber heading_sub;
};
heading_controller::heading_controller(){
	heading_sub = n.subscribe<rxtx_server::heading_pos>("heading_control", 1, &heading_controller::heading_controllerCallback, this);
}

void heading_controller::heading_controllerCallback(const rxtx_server::heading_control::ConstPtr&  heading){
	yaw = heading->yaw;
}
*/
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
		unsigned int vertical;
		unsigned int prev;
		unsigned int horizontal_1;
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
	vertical = ((2 * lidardata->vertical) + vertical)/3;
	horizontal_1 = lidardata->horizontal_1;
	horizontal_1 = ((2 * lidardata->horizontal_1) + horizontal_1)/3;

	if(vertical > 120){
		vertical = 0;
	}

	if(vertical > (prev + 5)){
		vertical = prev + 5;
	}
	else if(prev > 5){
		if(vertical < (prev - 5)){
			vertical = prev - 5;
		}
	}
	prev = vertical;
	
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

class altitude_handler{
	public:
		altitude_handler();
		int throttle_little;
		int throttle_big;
		char status;
	private:
		void altitude_handlerCallback(const osuar_telepilot::altitude_command::ConstPtr& altitude_command_data);
		ros::NodeHandle n;
		ros::Subscriber altitude_handler_sub;
};
altitude_handler::altitude_handler(){
	altitude_handler_sub = n.subscribe<osuar_telepilot::altitude_command>("altitude_info", 1, &altitude_handler::altitude_handlerCallback, this);
}

void altitude_handler::altitude_handlerCallback(const osuar_telepilot::altitude_command::ConstPtr& altitude_command_data){
	throttle_little = altitude_command_data->throttle_little;
	throttle_big= altitude_command_data->throttle_big;
	status = altitude_command_data->status;

	mvprintw(21,0,"RECEIVING THROTTLE_LITTLE %4d", throttle_little);
	mvprintw(22,0,"RECEIVING THROTTLE_BIG %4d", throttle_big);
}

class wall_handler{
	public:
		wall_handler();
		int tilt;
		char status;
	private:
		void wall_handlerCallback(const osuar_telepilot::wall_command::ConstPtr& wall_command_data);
		ros::NodeHandle n;
		ros::Subscriber wall_handler_sub;
};
wall_handler::wall_handler(){
	wall_handler_sub = n.subscribe<osuar_telepilot::wall_command>("wall_info", 1, &wall_handler::wall_handlerCallback, this);
}

void wall_handler::wall_handlerCallback(const osuar_telepilot::wall_command::ConstPtr& wall_command_data){
	tilt = wall_command_data->tilt;
	status = wall_command_data->status;

	mvprintw(21,40,"RECEIVING TILT %4d\n", tilt);
}


