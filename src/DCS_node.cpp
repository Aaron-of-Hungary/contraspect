/*
ROS DCS_node.cpp - ~/catkin_ws/src/contraspect/src/DCS_node.cpp
Author: 
 Anis Koubaa, Gaitech EDU
Project: 
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "DCS_node");
	ros::NodeHandle n;
	ros::Publisher pub_DCS_cmd = n.advertise<std_msgs::String>("DCS_cmd", 1000);
	std_msgs::String msg;
	std::stringstream ss;
	ss << argv[1];
	msg.data = ss.str();
	ROS_INFO("[DCS_node] Published to DCS_cmd: \"%s\"", msg.data.c_str());
	pub_DCS_cmd.publish(msg);
	
   return 0;
}

/* 
Exit
 /media/george/3218A8F718A8BAED/Users/hajdu/BME/ONL/ROS-projects 
*/
