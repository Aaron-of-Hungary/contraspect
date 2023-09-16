/***
  Title: contraspect_cpp
  Author: Aron Hajdu-Moharos
  Filename: contraspect/src/contraspect.cpp
  Description: Function definitions for the contraspect/contraspect.h library
  Project: Contraspect Drone Location and Navigation System
***/

#include "contraspect/contraspect.h"
#include <cstdlib>
#include <curses.h>
#include <sstream>
#include <string>
#include <fstream>
#include <cmath>

constexpr unsigned MAXFSIZE  = 4000; /* Max toread txt file size, def 4000 */
constexpr double EPSILON = 0.1;

namespace cspect{
  unsigned lastdigit_msec(double X){
    X = X*1000; /* Input in seconds, convert to milliseconds */
    X = std::fmod(X,10.0); /* Take last digit */
    return static_cast<unsigned>(std::abs(X)); /* return unsigned */
  }
  
  char getch_nodelay() {
    ::initscr();  /* Initialize ncurses */
    ::cbreak();   /* Disable line buffering */
    ::noecho();   /* Disable echoing of characters */
    keypad(stdscr, TRUE);  /* Enable special keys */
    curs_set(0);  /* Hide cursor */
    nodelay(stdscr, TRUE);  /* Enable non-blocking input */
    char ch = getch();  /* Get character without blocking */
    endwin(); /* Deinitialize ncurses */
    return ch;
  }
  void readFromKeyboard(double *Move){
    switch(cspect::getch_nodelay()){
    case 'q': Move[2] = Move[2] - 1.0; break;
    case 'w': Move[1] = Move[1] + 1.0; break;
    case 'e': Move[2] = Move[2] + 1.0; break;
    case 'a': Move[0] = Move[0] - 1.0; break;
    case 's': Move[1] = Move[1] - 1.0; break;
    case 'd': Move[0] = Move[0] + 1.0; break;
    default: return;
    }
  }
  
  void normalizeDouble3(double *Move){
    /* Normalize Move to absolute value 1 */
    double absVal = 0.0;
    for(size_t ii = 0; ii!=3; ii++) absVal = absVal + pow(Move[ii], 2.0);
    absVal = pow(absVal, 0.5); /* absVal = RMS (M1,M2,M3) */
    if(absVal > EPSILON) for(size_t ii = 0; ii!=3; ii++) Move[ii] = Move[ii]/absVal;
    else /*ROS_WARN("normalizeDouble3 tried to run with NULL input.")*/;    
  }

  std::string capitalize(char* X){
    std::string s = X;
    return capitalize(s);
  }
  std::string capitalize(std::string &X){ 
    std::string res;
    for(size_t ii = 0; ii!=X.length(); ii++)
      if(X[ii]>='a' && X[ii]<='z')
	res += X[ii] - ('a'-'A');
      else
	res += X[ii];
    return res;
  }
  
  unsigned ROS_INFO_F(const char* Filename){
    std::ifstream IF(Filename);
    if(!IF){
      ROS_ERROR("Loading file %s failed.", Filename);
      return 1;
    }
    IF.seekg(0,std::ios::end);
    unsigned fSize = IF.tellg();
    if(fSize > MAXFSIZE){
      ROS_ERROR("File too large. Max file size: %d", MAXFSIZE);
      return 1;
    }
    IF.seekg(0, std::ios::beg);
    char *cstr = new char[fSize+1];
    IF.read(cstr, fSize);
    cstr[fSize] = '\0';
    IF.close();
    ROS_INFO("%s", cstr);
    delete[] cstr;
    return 0;
  }  
  
  void ssAddFront(const char* C, std::stringstream &ss){
    std::string X = ss.str();
    ss.str("");
    cspect::ssEnd(ss);
    ss << C << X;
  }
  void ssEnd(std::stringstream &ss){
    ss.seekp(0, std::ios_base::end);
  }

  
  double dist3D(double X[3], double Y[3]){
    return sqrt(pow(X[0]-Y[0],2.0)+pow(X[1]-Y[1],2.0)+pow(X[2]-Y[2],2.0));
  }
  double arg3D(double X[3]){
    return sqrt(pow(X[0],2.0)+pow(X[1],2.0)+pow(X[2],2.0));
  }
  
}

