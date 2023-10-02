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
  size_t IMAX = cspect::BEACONS_NUM+2;

  if(argc > 1){
    demo = true;
    IMAX++;
  }
  
  
  for(size_t ii=0; ii!=IMAX; ii++){
    ss.str("");
    
    ss << "gnome-terminal --tab --title \"";    
    if(ii==0) ss << "DCS-B";
    else if(ii==1) ss << "Dn";
    else if(ii==cspect::BEACONS_NUM+2) ss << "Loc_sim";
    else ss << "B" << (int)ii-1;
    
    ss << "\" -- bash -c \"rosrun contraspect";    
    if(ii==0) ss << " Dcs_node B";
    else if(ii==1) ss << " Dn_node";
    else if(ii==cspect::BEACONS_NUM+2) ss << "_demo Loc_sim";
    else ss << " Beacon_node " << (int)ii-1;

    if(ii!=cspect::BEACONS_NUM+2 && ii!=0)
      for(size_t jj=0; jj!=3; jj++) ss << ' ' << std::to_string(INIT_COORDS[ii-1][jj]);
    if(demo && ii==1) ss << " demo";
    
    ss <<"; exec bash\"";
    s = ss.str();
    if(system(s.c_str())) ROS_ERROR("Fail: %s", s.c_str());
    else ROS_INFO("Execute: %s",s.c_str());
  }
  return 0;
}
