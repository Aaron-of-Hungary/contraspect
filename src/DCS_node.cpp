/*
ROS DCS_node.cpp - ~/catkin_ws/src/contraspect/src/DCS_node.cpp
Author: 
 Anis Koubaa, Gaitech EDU
Project: 
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

void triangCallback(const std_msgs::String::ConstPtr& topic_msg){
    ROS_INFO("[DCS_node] Heard from 'DCS-Stream': \"%s\"", topic_msg->data.c_str());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "DCS_node");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("DCS-Stream", 1000, triangCallback);
    ros::spin(); // does it spin ad infinitem? If yes, good. It is to be turned off manually
    return 0;
}

/* 
Exit
 /media/george/3218A8F718A8BAED/Users/hajdu/BME/ONL/ROS-projects 
*/
