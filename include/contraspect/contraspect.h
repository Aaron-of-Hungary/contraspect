/***
  Title: contraspect_h
  Author: Aron Hajdu-Moharos
  Filename: contraspect/include/contraspect/contraspect.h
  Description: Function declarations for the contraspect/contraspect.h library
  Project: Contraspect Drone Location and Navigation System
***/

#ifndef CONTRASPECT_CONTRASPECT_H
#define CONTRASPECT_CONTRASPECT_H

#include "ros/ros.h"
#include <sstream>
#include <string>
namespace cspect {

  /***********************************************************************/
  /*************//*    ADJUST THESE VALUES AS NEEDED   *//****************/
  /***********************************************************************/
  constexpr unsigned SUBRATE      = 1000,  /* def 1000			 */
  /**/	  	     PUBRATE      = 1000,  /* def 1000 			 */
  /**/  	     BEACONS_NUM  = 4;     /* WILL BREAK IF NOT FOUR	 */
  constexpr double   SPEEDOFSOUND = 34.30, /* in air, room temp: 343 m/s */
  /**/		     /* Minimum Drone position resolution in meters:	 */
  /**/		     MINRES	  = 0.1,       		     	       /**/
  /**/		     /* Added weight (eg. SNR) of beacons:	         */
  /**/		     BEACONS_WEIGHT[BEACONS_NUM] = {1.0,1.0,1.0,1.0},  /**/
  /**/		     /* Clk Sync frequency [Hz] for Bcns, Drone, DCS-B:	 */
  /**/		     CLK_SYNC_FREQ = 1.0; 	    	  	       /**/
  /***********************************************************************/
  /***********************************************************************/

  /***********************************************************************/
  /***************//*    DO NOT TOUCH THESE VALUES   *//******************/
  /***********************************************************************/  
  /* Maximum allowed delay to achieve min resolution:			 */
  constexpr double   MAXDELAY 	  = MINRES/SPEEDOFSOUND,	       /**/
  /* Loop period for all nodes. (All Bcns publish simultaneously!!) :	 */
  /**/ 	    	     PERIOD	  = MAXDELAY/BEACONS_NUM;	       /**/
  /* So many PERIODs add up to 10 milliseconds:				 */
  constexpr unsigned P_10ms 	  = (unsigned)(0.01/PERIOD+1.0);       /**/	 
  /***********************************************************************/
  /***********************************************************************/

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
