/***
  Title: Init
  Author: Aron Hajdu-Moharos
  Filename: contraspect/src/Init.cpp
  Description: Node to automatically initialize Drone and Beacons
  Project: Contraspect Drone Location and Navigation System
***/

#include <contraspect/contraspect.h>
#include <sstream>
#include <cstdlib>

/* EDIT AS NEEDED: */
constexpr double INIT_COORDS[cspect::BEACONS_NUM+1][3] =
  {{.0,.0,1.0},{-1.0,.0,.0},{.0,-1.0,.0},{.0,1.0,.0},{1.0,.0,.1}}; 

void execute(std::string &);

int main(int argc, char **argv){

  std::stringstream ss;
  std::string s;
  bool demo = false;

  if(argc > 1) demo = true;  
  
  for(size_t ii=0; ii!=cspect::BEACONS_NUM+1; ii++){
    ss.str("");
    ss << "gnome-terminal --tab -- bash -c \"rosrun contraspect ";
    if(ii==0) ss << "Dn_node";
    else ss << "Beacon_node " << (int)ii;
    for(size_t jj=0; jj!=3; jj++) ss << ' ' << std::to_string(INIT_COORDS[ii][jj]);
    if(demo && ii==0) ss << " demo";
    ss <<"; exec bash\"";
    s = ss.str();
    if(system(s.c_str())) ROS_ERROR("Fail: %s", s.c_str());
    else ROS_INFO("Execute: %s",s.c_str());
  }
  return 0;
}
