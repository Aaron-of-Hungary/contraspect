/*
ROS beacon_node.cpp - ~/catkin_ws/src/contraspect/src/beacon_node.cpp
Author: 
 Aron Hajdu-Moharos
Project title:
 ContraSpect Drone-Based Location- and Navigation System 
*/

#include "ros/ros.h"
#include "ros/master.h"
#include "ros/this_node.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <cstring>
#include <cmath>

// function to ask ros master if a node called node_name is running. ros::master::getNodes() not working - FIX!
bool isNodeRunning(const std::string& node_name);
// function to check whether str has format "12.345" or "67,890" or neither
bool floatFormat(const char* str);

// argv: unique_nodename xcoord ycoord zcoord
int main(int argc, char **argv)
{
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*\
   || Initializing a beacon_node, setting 3D location-coordinates ||
  \*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  std::string tmpstring;
  std::string nodename = argv[1];      
  double coord_array[3] = {0.0, 0.0, 0.0}; // x, y and z
	// checking if there exists a node running called nodename	
	if (0 /*isNodeRunning(nodename)*/) {
	  ROS_ERROR("Node with name '%s' is already running. Try a different unique nodename.", nodename.c_str());
	  return 1;
	} 
	// initializing coordinates - ADD option for minus sign!
	for(size_t ii(2); ii!=5; ii++){
	  tmpstring.clear();
	  tmpstring += argv[ii];	
	  if(!floatFormat(tmpstring.c_str())){ // checking if correct coordinate format
	    ROS_ERROR("Wrong coordinate format: %s \n Correct format: 12.345 or 678,90", tmpstring.c_str());
	    return 1;  
	  }
	  size_t jj(0), kk(0);
	  for(; tmpstring[jj]!='.' && tmpstring[jj]!=','; jj++); // finding decimal point index
	  // string to double conversion:
	  for(; kk!=jj; kk++){
	    int xx(0), yy(0), zz(0);
	    double ww(0.0);
	    xx = pow(10, kk);
	    yy = (int)( tmpstring[jj-1-kk] - ('0'-'\0') );
	    zz = xx * yy;
	    ww = coord_array[ii-2];
	    coord_array[ii-2] = ww + (double)zz;
	    //ROS_INFO("ii = %ld || kk = %ld || %f + %f = %f", ii, kk, ww, (double)zz, coord_array[ii-2]); // <- for debugging string to double conversion
	  }
	  kk++; // skip decimal point
	  for(; kk!=tmpstring.length(); kk++){
	    double xx(0.0), zz(0.0), ww(0.0);
	    int yy(0);
	    xx = pow( 10.0, ((double)jj-(double)kk) );
	    yy = (int)( tmpstring[kk] - ('0'-'\0') );
	    zz = xx * (double)yy;
	    ww = coord_array[ii-2];
	    coord_array[ii-2] = ww + zz;
	    //ROS_INFO("ii = %ld || kk = %ld || %f + %f = %f", ii, kk, ww, zz, coord_array[ii-2]); // <- for debugging string to double conversion
	  }
	}
	// initializing node with unique nodename
	ROS_INFO("Initializing beacon_node.\t Unique-name: \t %s \n\t\t\t\t\t\t\t\t X-coord: \t %f \n\t\t\t\t\t\t\t\t Y-coord: \t %f \n\t\t\t\t\t\t\t\t Z-coord: \t %f"
		 , nodename.c_str(), coord_array[0], coord_array[1], coord_array[2]);
	ros::init(argc, argv, nodename.c_str());
	if (!ros::master::check()) {
	  ROS_ERROR("Failed to initialize ROS.");
	  return 1;
	}
	ros::NodeHandle n;
	ros::Publisher triang_publisher = n.advertise<std_msgs::String>("triang", 1000);
	ros::Rate loop_rate(0.5); // spin once per second
   int count = 0;
   while (ros::ok()) 
   {
       std_msgs::String msg;
       std::stringstream ss;
       ss << "beacon_node \"" << nodename << "\" Triang Msg " << count/10;
       msg.data = ss.str();
       if(!(count%10)){ // publish once every 10 spins
	 ROS_INFO("[%s] msg published to topic \"triang\": %s\n", nodename.c_str(), msg.data.c_str());
	 triang_publisher.publish(msg);
       }
       // ROS_INFO("Spin once");
       ros::spinOnce(); 
       loop_rate.sleep();
       count++;
   }
   return 0;
}

// function to check if a node with a given name is running
bool isNodeRunning(const std::string& node_name) {
    std::vector<std::string> nodes;
    ROS_INFO("A");
    ros::master::getNodes(nodes);
    ROS_INFO("B");
    for (const auto& node : nodes) {
        if (node == node_name) {
            return true;
        }
    }
    return false;
}

// function to check whether str has format "12.345" or "67,890" or neither
bool floatFormat(const char* str) {
    bool hasDecimal = false;
    size_t length = strlen(str);
    for (size_t ii = 0; ii != length; ++ii) {
      if (str[ii] == '.' || str[ii] == ',') {
	if (hasDecimal) {
	  // Found a second decimal point
	  return false;
	}
	hasDecimal = true;
      } else if (str[ii] < '0' || str[ii] > '9') {
	// Found a character that is not a digit or decimal point
	return false;
      }
    }
    return true;
}

/* 
Exit
 /media/george/3218A8F718A8BAED/Users/hajdu/BME/ONL/ROS-projects 
*/
