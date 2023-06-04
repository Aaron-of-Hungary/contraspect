/*

ROS Dn.cpp - ~/catkin_ws/src/contraspect/src/Dn.cpp

Author: 
 Anis Koubaa, Gaitech EDU
Project: 

 */

#include "ros/ros.h"
#include "std_msgs/String.h"

void triangCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("[Dn] Msg from topic triang: [%s]\n", msg->data.c_str());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Dn_node");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("triang", 1000, triangCallback);
    ros::spin();
    return 0;
}

/* 

Exit
 /media/george/3218A8F718A8BAED/Users/hajdu/BME/ONL/ROS-projects 

*/
