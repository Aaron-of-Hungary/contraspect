/*

ROS B2.cpp - ~/catkin_ws/src/contraspect/src/B2.cpp

Author: 
 Anis Koubaa, Gaitech EDU

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "B2_node");
	ros::NodeHandle n;
	ros::Publisher chatter_publisher = n.advertise<std_msgs::String>("triang", 1000);
	ros::Rate loop_rate(0.25); //1 message per 2 seconds

   int count = 0;
   while (ros::ok()) 
   {
       std_msgs::String msg;
       std::stringstream ss;
       ss << "B2 Triang Msg " << count;
       msg.data = ss.str();
       ROS_INFO("[B2] msg published to topic \"triang\": %s\n", msg.data.c_str());
       chatter_publisher.publish(msg);
       ros::spinOnce(); 
       loop_rate.sleep();
       count++;
   }
   return 0;
}

/* 

Exit
 /media/george/3218A8F718A8BAED/Users/hajdu/BME/ONL/ROS-projects 

*/
