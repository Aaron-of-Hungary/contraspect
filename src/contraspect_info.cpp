/*
ROS contraspect_info.cpp - ~/catkin_ws/src/contraspect/src/contraspect_info.cpp
Author:
 Aron Hajdu-Moharos
Project title:
 Contraspect Drone-Based Location- and Navigation System
Program Function:
 Lists information about the contraspect package
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
	// Initiate new ROS node named "contraspect_info"
	ros::init(argc, argv, "contraspect_info");

  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
   || Print info regarding contraspect package ||
  \*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	
	//create a new String ROS message.
	std_msgs::String msg;
	
	//create a string for the data
	std::stringstream ss;
	ss << "\n\n"
	   << "Contraspect dron- es bazisallomas-alapu helymeghatarozo- es navigacios rendszer\n"
	   << "-- package=contraspect --\n"
	   << "-- a rendszer alapfunkcioit ellato csomag\n"	  
	   << "\n"
	   << "Futtathato node-ok:\n"
	   << "\n"
	   << "$ rosrun contraspect DCS_node cmd\n"
	   << "# A digitalis iranyitoallomast szimulalo egyseg.\n"
	   << "# cmd opciok:\n"
	   << "  # init-trigger - az inicializaciot megelozo protokoll\n"
	   << "  # init - az inicializacios protokoll\n"
	   << "  # move x y z - a dron manualis mozgatasa x y z pozicioba\n"
	   << "\n"
	   << "$ rosrun contraspect Dn_node x y z\n"
	   << "# A dront szimulalo egyseg.\n"
	   << "  # x y z - a kezdeti pozicio koordinatai\n"
	   << "\n"
	   << "$ rosrun contraspect beacon_node uname x y z\n"
	   << "# A bazisallomasokat szimulalo egyseg.\n"
	   << "  # uname - a node egyedi neve - javasolt: B1_node, B2_node, B3_node, B4_node\n"
	   << "  # x y z - a kezdeti pozicio koordinatai - javasolt: 0 0 0; 0 1 0; 1 0 0; 1 1 0\n"
	   << "\n" 
	   << std::endl;

	//assign the string data to ROS message data field
	msg.data = ss.str();
	
	//print the content of the message in the terminal
	ROS_INFO("%s", msg.data.c_str());
	
	while (ros::ok()); // Keep spinning loop until user presses Ctrl+C
   
   return 0;
}

/* Exit
/media/george/3218A8F718A8BAED/Users/hajdu/BME/ONL/ROS-projects 
*/
