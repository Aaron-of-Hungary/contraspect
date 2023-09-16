/***
  Title: Contraspect_info
  Author: Aron Hajdu-Moharos
  Filename: contraspect/src/Contraspect_info.cpp
  Description: Node to display info on contraspect pkg and nodes
  Project: Contraspect Drone Location and Navigation System
***/

#include "ros/ros.h"
#include "contraspect/contraspect.h"
#include <sstream>

constexpr char TXTDIR[] = "/home/george/catkin_ws/src/contraspect/txt/";

enum nType {Naught, Dn_node, Beacon_node, Dcs_node, Contraspect_info};

unsigned argvParse(int &Argc, char **Argv, enum nType &ActiveNode);
std::string activeNode_str(enum nType &ActiveNode);

int main(int argc, char **argv){
 
  /* Parse argv */
  enum nType activeNode = Naught;
  if (argvParse(argc, argv, activeNode)){
    ROS_ERROR("Parsing argv failed. Node shutdown.");
    return 1;    
  }

  /* Initialize node */
  std::stringstream ss;
  if(activeNode) ss.str("_");
  cspect::ssEnd(ss);
  ss << activeNode_str(activeNode);
  cspect::ssAddFront("Contraspect_info", ss);
  std::string s = ss.str();
  ros::init(argc, argv, s.c_str());
  /*ROS_INFO("Initialize node success");*/
  
  /* Print node info */
  if(activeNode == Naught) ss.str("Contraspect");
  else ss.str(activeNode_str(activeNode));
  cspect::ssEnd(ss);
  ss << "_INFO.txt";
  cspect::ssAddFront(TXTDIR, ss);
  s = ss.str();
  if(cspect::ROS_INFO_F(s.c_str())){
    ROS_ERROR("Print %s failed. Node shutdown.", s.c_str());
    return 1;
  }
  return 0;
}

unsigned argvParse(int &Argc, char **Argv, enum nType &ActiveNode){
  /* Check argc */
  if(Argc<1 || Argc>2){
    ROS_ERROR("Parse argv error. Incorrect number of args.");
    return 1;
  }
  if(Argc == 1) {
    ActiveNode = Naught;
    return 0;
  }
  /* Parse argv */
  std::string s = cspect::capitalize(Argv[1]);
  if(	  s == "DN_NODE")		ActiveNode = Dn_node;
  else if(s == "BEACON_NODE") 		ActiveNode = Beacon_node;
  else if(s == "DCS_NODE") 		ActiveNode = Dcs_node;
  else if(s == "CONTRASPECT_INFO") 	ActiveNode = Contraspect_info;
  else{
    ROS_ERROR("Parse argv error. Incorrect format. Try: Dn_node, Beacon_node, Dcs_node, Contraspect_info");
    return 1;
  }
  return 0;
}

std::string activeNode_str(enum nType &ActiveNode) {
  std::string res;
  switch (ActiveNode) {
    case Naught: res = ""; break;
    case Dn_node: res = "Dn_node"; break;
    case Beacon_node: res = "Beacon_node"; break;
    case Dcs_node: res = "Dcs_node"; break;
    case Contraspect_info: res = "Contraspect_info"; break;
  }
  return res;
}
