/***
  Title: Contraspect_demo_info
  Author: Aron Hajdu-Moharos
  Filename: contraspect_demo/src/Contraspect_demo_info.cpp
  Description: Node to display info on contraspect_demo pkg and nodes
  Project: Contraspect Drone Location and Navigation System
***/

#include "ros/ros.h"
#include "contraspect/contraspect.h"
#include <sstream>

constexpr char TXTDIR[] = "/home/george/catkin_ws/src/contraspect_demo/txt/";

enum nType {Base, Topic_spy, Loc_sim, Contraspect_demo_info};

unsigned argvParse(int &Argc, char **Argv, enum nType &ActiveNode);
std::string activeNode_str(enum nType &ActiveNode);

int main(int argc, char **argv){
 
  /* Parse argv */
  enum nType activeNode = Base;
  if (argvParse(argc, argv, activeNode)){
    ROS_ERROR("Parsing argv failed. Node shutdown.");
    return 1;    
  }

  /* Initialize node */
  std::stringstream ss;
  if(activeNode) ss.str("_");
  cspect::ssEnd(ss);
  ss << activeNode_str(activeNode);
  cspect::ssAddFront("Contraspect_demo_info", ss);
  std::string s = ss.str();
  ros::init(argc, argv, s.c_str());
  /*ROS_INFO("Initialize node success");*/
  
  /* Print node info */
  if(!activeNode) ss.str("Contraspect_demo");
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
  if(Argc>2){
    ROS_ERROR("Parse argv error. Max argc 2");
    return 1;
  }
  if(Argc == 1) {
    ActiveNode = Base;
    return 0;
  }
  /* Parse argv */
  std::string s = cspect::capitalize(Argv[1]); 
  if(	  s == "TOPIC_SPY") 	  ActiveNode = Topic_spy;
  else if(s == "LOC_SIM") 	  ActiveNode = Loc_sim;
  else if(s == "CONTRASPECT_DEMO_INFO") ActiveNode = Contraspect_demo_info;
  else{
    ROS_ERROR("Parse argv error. Incorrect format.\nTry: Topic_spy, Loc_sim, Contraspect_demo_info");
    return 1;
  }
  return 0;
}

std::string activeNode_str(enum nType &ActiveNode) {
    switch (ActiveNode) {
        case Base: return "";
        case Topic_spy: return "Topic_spy";
        case Loc_sim: return "Loc_sim";
        case Contraspect_demo_info: return "Contraspect_demo_info";
        default: return "";
    }
}
