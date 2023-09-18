/**
  Title: contraspect_h
  Author: Aron Hajdu-Moharos
  Filename: contraspect/include/contraspect/contraspect.h
  Description: Function declarations for the contraspect/contraspect.h library
  Project: Contraspect Drone Location and Navigation System
**/

#ifndef CONTRASPECT_CONTRASPECT_H
#define CONTRASPECT_CONTRASPECT_H

#include "ros/ros.h"
#include <sstream>
#include <string>
namespace cspect {

  constexpr unsigned SUBRATE  = 1000,	   /* def 1000			 */
  	  	     PUBRATE  = 1000, 	   /* def 1000 			 */
    		     BEACONS_NUM  = 4;     /* def 4 			 */
  constexpr double   SPEEDOFSOUND = 343.0, /* in air, room temp: 343 m/s */
                     /* NONZERO POSITIVE! Beacons Triang loop period.    */
                     BPERIOD      = 0.0003,
  		     /* Dn, Dcs, Loc_sim loop period	      		 */
  		     /* Due to all Beacons constantly publishing: 	 */
    		     DPERIOD      = BPERIOD/BEACONS_NUM;
		     /* So many DPERIODs add up to 10 milliseconds:	 */
  constexpr unsigned D_10ms 	  = (unsigned)(0.01/DPERIOD+1.0);

  unsigned lastdigit_msec(double X);
  unsigned ROS_INFO_F(const char *Filename);
  std::string capitalize(char* X);
  std::string capitalize(std::string &);
  char getch_nodelay();
  void readFromKeyboard(double *Move);
  void normalizeDouble3(double *Move);
  void ssAddFront(const char* C, std::stringstream &ss);
  void ssEnd(std::stringstream &ss);
  double dist3D(double[3], double[3]);
  double arg3D(double[3]);

  
}
#endif /*CONTRASPECT_CONTRASPECT_H*/
