/*

ROS Dn_node.cpp - ~/catkin_ws/src/contraspect/src/Dn_node.cpp

Author: 
 Anis Koubaa, Gaitech EDU
Project: 

 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

double pos[3] = {0.0, 0.0, 0.0};
double beacon_dist[4] = {0.0, 0.0, 0.0, 0.0};
bool init = false;

// function to check whether str has format "12.345" or "67,890" or neither. Auxiliary of coordParse()
bool floatFormat(const char* str);
// function to parse coordinates from argv
void coordParse(double (&coord_array)[3], std::string& tmpstring, size_t& ii);
// Callback function upon receiving triang message
void callback_triang(const std_msgs::String::ConstPtr& msg);
// Callback function upon receiving triang_delay_info message
void callback_triang_delay_info(const std_msgs::String::ConstPtr& msg);
// Callback function upon receiving DCS_cmd message - forward to Dn_cmd
void callback_DCS_cmd(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv){
    ros::init(argc, argv, "Dn_node");
    ros::NodeHandle node;
    ros::Subscriber sub_triang = node.subscribe("triang", 1000, callback_triang);
    ros::Subscriber sub_DCS_cmd = node.subscribe("DCS_cmd", 1000, callback_DCS_cmd);
    ros::Subscriber sub_triang_delay_info
      = node.subscribe("triang_delay_info", 1000, callback_triang_delay_info);

    ros::Publisher pub_DCS_recv_data = n.advertise<std_msgs::String>("DCS_recv_data", 1000);
    ros::Publisher pub_loc_track_data = n.advertise<std_msgs::String>("loc_track_data", 1000);
    ros::Publisher pub_triang_calc_result
      = n.advertise<std_msgs::String>("triang_calc_result", 1000);
    ros::Publisher pub_triang_recv_data
      = n.advertise<std_msgs::String>("triang_recv_data", 1000);
    ros::Publisher pub_Dn_cmd = n.advertise<std_msgs::String>("Dn_cmd", 1000);

    
    ros::spin();
    return 0;
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

// function to parse coordinates from argv input string
void coordParse(double (&coord_array)[3], std::string& tmpstring, size_t& ii){
  //ROS_INFO("parse begin");
  size_t jj(0), kk(0);
  for(; tmpstring[jj]!='.' && tmpstring[jj]!=','; jj++); // finding decimal point index
  // string to double conversion:
  for(; kk!=jj; kk++){
    int xx(0), yy(0), zz(0);
    double ww(0.0);
    xx = pow(10, kk);
    yy = (int)( tmpstring[jj-1-kk] - ('0'-'\0') );
    zz = xx * yy;
    ww = coord_array[ii];
    coord_array[ii] = ww + (double)zz;
    //ROS_INFO("ii = %ld || kk = %ld || %f + %f = %f", ii, kk, ww, (double)zz, coord_array[ii]); // <- for debugging string to double conversion
  }
  kk++; // skip decimal point
  for(; kk!=tmpstring.length(); kk++){
    double xx(0.0), zz(0.0), ww(0.0);
    int yy(0);
    xx = pow( 10.0, ((double)jj-(double)kk) );
    yy = (int)( tmpstring[kk] - ('0'-'\0') );
    zz = xx * (double)yy;
    ww = coord_array[ii];
    coord_array[ii] = ww + zz;
    //ROS_INFO("ii = %ld || kk = %ld || %f + %f = %f", ii, kk, ww, zz, coord_array[ii]); // <- for debugging string to double conversion
  }
}

// Callback function upon receiving triang message - Do nothing
void callback_triang(const std_msgs::String::ConstPtr& msg){
  // ROS_INFO("[Dn_node] Triang-msg received: \"%s\"", msg->data.c_str());
  ;
}

// Callback function upon receiving triang_delay_info message - calculate beacon distances
void callback_triang_delay_info(const std_msgs::String::ConstPtr& msg){
  ;
}
// Callback function upon receiving DCS_cmd message - forward to Dn_cmd
void callback_DCS_cmd(const std_msgs::String::ConstPtr& msg){
  // case init-trigger protocol
  if(!strcmp(msg.data.c_str(), "init_trigger")){
    // fly up
    pos[2] = pos[2] + 1.5;
    // notify DCS
    ros::Publisher pub_DCS_recv_data = n.advertise<std_msgs::String>("DCS_recv_data", 1000);
    std_msgs::String msg_out;
    std::stringstream ss;
    ss << "[Dn_node] init_trigger protocol executed, ready for init protocol.";
    msg_out.data = ss.str();
    pub_DCS_recv_data.publish(msg_out);
  }  
  // case init protocol
  else if(!strcmp(msg.data.c_str(), "init")){
    init = true;
    // notify DCS
    ros::Publisher pub_DCS_recv_data = n.advertise<std_msgs::String>("DCS_recv_data", 1000);
    ros::Publisher pub_Dn_cmd = n.advertise<std_msgs::String>("Dn_cmd", 1000);
    std_msgs::String msg_out;
    std::stringstream ss;
    ss << "[Dn_node] init protocol executed, ready for move protocol.";
    msg_out.data = ss.str();
    pub_DCS_recv_data.publish(msg_out);
    // notify beacons
    pub_Dn_cmd.publish(msg);
  }
  // case move protocol
  else{
    size_t ii = 0;
    ros::Publisher pub_DCS_recv_data = n.advertise<std_msgs::String>("DCS_recv_data", 1000);
    std_msgs::String msg_out;
    std::stringstream ss;
    for(; ii< msg.data.length() && msg.data[ii] != ' '; ii++)
      ss << msg.data[ii];
    if(!strcmp(ss.str().c_str(), "move")){
      for(size_t jj = 0; jj!=3; jj++){
	ii++; // skip whitespace
	ss.clear();
	for(; ii< msg.data.length() && msg.data[ii] != ' '; ii++) // read coord param from msg
	  ss << msg.data[ii];
	if(!floatFormat(ss.str().c_str())){ // checking if correct coordinate format
	  ROS_ERROR("Wrong coordinate format: %s \n Correct format: 12.345 or 678,90",
		    ss.str().c_str());
	    return 1;  
	  }
	coordParse(pos, ss.str().c_str(), jj);	  
      }
    } else {
      ROS_ERROR("Topic DCS_cmd: Wrong command format published by DCS.");
      return 1;
    }

  }
  
}

/* 
Exit
 /media/george/3218A8F718A8BAED/Users/hajdu/BME/ONL/ROS-projects 
*/