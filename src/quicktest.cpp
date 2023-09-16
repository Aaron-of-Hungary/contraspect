/***
  Title: quicktest
  Author: Aron Hajdu-Moharos
  Filename: contraspect/src/quicktest.cpp
  Description: Node to test cspect::keyboard() and std::system() functions
  Project: Contraspect Drone Location and Navigation System
***/

#include <ros/ros.h>
#include <cstdlib>
#include <random>
#include <contraspect/contraspect.h>

constexpr double PERIOD = 1.0;

int main(int argc, char **argv) {

  /* Initialize node */
  ros::init(argc, argv, "quicktest");
  ros::NodeHandle nh;
  
  /* Initialize variables */
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(0.0, 10000.0);
  double x;
  unsigned y;
  
  /* Loop begin */
  ros::Rate loopRate(pow(PERIOD, -1.0));  // Control rate in Hz
  while (ros::ok()) {

    x = dis(gen);
    y = cspect::lastdigit_msec(x);
    ROS_INFO("Random double value: %fl\n\t\t\t\tThird digit post decimal:   %u", x,y);
    
    /* Loop end */
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}

