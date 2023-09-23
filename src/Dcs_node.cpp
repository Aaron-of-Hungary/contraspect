/***
  Title: Dcs_node
  Author: Aron Hajdu-Moharos
  Filename: contraspect/src/Dcs_node.cpp
  Description: Simulation node of the Digital Control Station (DCS)
  Project: Contraspect Drone Location and Navigation System
***/

#include "ros/ros.h"
#include "contraspect/contraspect.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "contraspect_msgs/Clk_sync.h" 
#include "contraspect_msgs/Status_map.h" 
#include "contraspect_msgs/Dn_ctrl.h" 
#include "std_msgs/String.h"
#include <sstream>
#include <cmath>

/* Constants declare */
constexpr char FILENAME[] = "/home/george/catkin_ws/src/contraspect/txt/Dcs_node_info.txt",
      	       DSM[]      = "Dn_status_map",
  	       OM[]	  = "object_markers",
 	       BASHCMD[]  = "gnome-terminal --tab -- bash -c \"rosrun rviz rviz; exec bash\"";
constexpr double EPSILON     = 0.001,
  MARKERSCALE = 0.1,                           /* Dn step scale, def 0.1                      */
  MAX_Z_VALUE = 5.0;                           /* Max height Dn reach, Min 1.0 Max 30.0       */

/* Global vars declare */
double status_map[cspect::BEACONS_NUM+2][3];
enum M {A,B,C}; /* A: recv Dn_status_map, B: send Clk_sync C: send Dn_ctrl */
enum M mode;
float  clk = 0.0;

/* Callback declare */
void callback_Dn_status_map(const contraspect_msgs::Status_map::ConstPtr &msg);
bool callback_Clk_sync	   (	  contraspect_msgs::Clk_sync  ::Request  &req,
		       	   	  contraspect_msgs::Clk_sync  ::Response &res);

/* Functions declare */
unsigned argvParse(int &Argc, char **Argv, enum M &Mode);
char modeChar(enum M &Mode);
void open3DMap();
void sendTo3DMap(double V[][3], ros::Publisher &Pub_Ojbect_Markers, const unsigned &ArrSize);

int main(int argc, char **argv){

  /* Print node info */
  if(cspect::ROS_INFO_F(FILENAME)) ROS_WARN("Print node info failed.");

  /* Parse argv */
  if(argvParse(argc, argv, mode)){
    ROS_ERROR("Parse argv failed. Node shutdown.");
    return 1;
  }
  /*ROS_INFO("Parse argv success");*/

  /* Initialize node and pub sub variables */
  std::stringstream ss;
  ss << "Dcs_node_" << modeChar(mode);
  std::string s = ss.str();
  ros::init(argc, argv, s.c_str());
  ros::NodeHandle n;
  ros::Subscriber sub_Dn_status_map;
  ros::Publisher pub_object_markers;
  ros::ServiceClient cli_Dn_ctrl;
  ros::ServiceServer ser_Clk_sync;

  /* Initialize MODE-specific pub-sub variables */
  if(mode==A){
    /*open3DMap();*/ /* Open rviz in new terminal tab */
    sub_Dn_status_map = n.subscribe(DSM, cspect::SUBRATE, callback_Dn_status_map);
    pub_object_markers  = n.advertise<visualization_msgs::MarkerArray>(OM, 10);
  }
  else if(mode==B){
    ser_Clk_sync = n.advertiseService("Clk_sync", callback_Clk_sync);
  }
  else if(mode==C) cli_Dn_ctrl  = n.serviceClient<contraspect_msgs::Dn_ctrl >("Dn_ctrl" );
  /*ROS_INFO("Initialize node and pub sub variables success");*/

  /* Initialize additional variables */
  size_t cntr = 0,				    /* loop counter			      */
    cSyncd_date[cspect::BEACONS_NUM+1]={0,0,0,0,0}, /* B: cntr_tstmps of srv_Clk_sync rspnses */
    send_next = 0, 			       	    /* B: oldest Clk_sync response src idx    */
    syncwin = cspect::BEACONS_NUM+1;                /* Sync window for Clk_sync	       	      */
  double    move[3]  = {0.0,0.0,0.0},               /* move cmd for Dn_ctrl 		      */
    ctrl_prev[3] = {0.0,0.0,0.0},		    /* stored previous value of Dn_ctrl	      */
    v[cspect::BEACONS_NUM+2][3];                    /* Dn_status_map recv data to visualize   */
  contraspect_msgs::Dn_ctrl srv_Dn_ctrl;
  
  /*Loop begin */
  ros::Rate loopRate(pow(cspect::PERIOD, -1.0));
  while(ros::ok()){

    /* Publish Dn & Bcns visualize data to object_markers for Rviz - Publish speed: 100Hz */
    if(mode == A){
      if(!(cntr%cspect::P_10ms)){
	for(size_t ii = 0; ii!=3; ii++)
	  v[0][ii] = status_map[0][ii] + status_map[1][ii]; /* Dn pos + status = Dn future pos */
	for(size_t ii = 1; ii!=cspect::BEACONS_NUM+2; ii++)
	  for(size_t jj = 0; jj!=3; jj++)
	    v[ii][jj] = status_map[ii][jj];
	sendTo3DMap(v, pub_object_markers, cspect::BEACONS_NUM+2);
	/*ROS_INFO("Pub to object_markers");*/
      }
    }

    /* Set internal clock to sync Drone and Beacons */
    else if(mode == B){
      if(clk>2048.0-cspect::PERIOD) clk = clk-2048.0+cspect::PERIOD;
      else clk = clk + cspect::PERIOD;      
    }  
    
    /* Call Dn_ctrl service */
    else if(mode == C){
      /* read 3 input chars to get complete direction */
      for(size_t ii = 0; ii!=3; ii++) cspect::readFromKeyboard(move);
      cspect::normalizeDouble3(move); /* Normalize to size 1 */
      /* store prior ctrl values */
      ctrl_prev[0] = srv_Dn_ctrl.request.x;
      ctrl_prev[1] = srv_Dn_ctrl.request.y;
      ctrl_prev[2] = srv_Dn_ctrl.request.z;
      /* set new ctrl values */
      if(cspect::arg3D(move) > EPSILON){ /* If nonzero input */
	srv_Dn_ctrl.request.x = srv_Dn_ctrl.request.x + move[0];
	srv_Dn_ctrl.request.y = srv_Dn_ctrl.request.y + move[1];
	srv_Dn_ctrl.request.z = srv_Dn_ctrl.request.z + move[2];
	ROS_INFO("%f\t%f\t%f\n\t\t\t +\t%f\t%f\t%f\n\t\t\t=\t%f\t%f\t%f",
		 ctrl_prev[0], ctrl_prev[1], ctrl_prev[2], move[0], move[1], move[2],
		 srv_Dn_ctrl.request.x, srv_Dn_ctrl.request.y, srv_Dn_ctrl.request.z);
	for(size_t ii = 0; ii!=3; ii++) move[ii] = 0.0;
	if(cli_Dn_ctrl.call(srv_Dn_ctrl)){
	  ROS_INFO("srv_Dn_ctrl call.");
	  srv_Dn_ctrl.request.z = srv_Dn_ctrl.request.y = srv_Dn_ctrl.request.x = 0.0;
	}else ROS_ERROR("srv_Dn_ctrl call fail.");
      }
    }

    /* Loop end */
    else break;
    ros::spinOnce();
    loopRate.sleep();
    cntr = cntr + 1;
  }

  /* End of code */
  return 0;
}

void callback_Dn_status_map(const contraspect_msgs::Status_map::ConstPtr &msg){
  if(msg){
    /* We assume beaconsNum == 4 */
    status_map[0][0] = msg-> sx; status_map[0][1] = msg-> sy; status_map[0][2] = msg-> sz;
    status_map[1][0] = msg-> dx; status_map[1][1] = msg-> dy; status_map[1][2] = msg-> dz;
    status_map[2][0] = msg->b1x; status_map[2][1] = msg->b1y; status_map[2][2] = msg->b1z;
    status_map[3][0] = msg->b2x; status_map[3][1] = msg->b2y; status_map[3][2] = msg->b2z;
    status_map[4][0] = msg->b3x; status_map[4][1] = msg->b3y; status_map[4][2] = msg->b3z;
    status_map[5][0] = msg->b4x; status_map[5][1] = msg->b4y; status_map[5][2] = msg->b4z;
    ROS_INFO("callback_Dn_status_map");
  }
}

bool callback_Clk_sync(contraspect_msgs::Clk_sync::Request  &req,
		       contraspect_msgs::Clk_sync::Response &res){
  res.adjust = clk-req.clk;
  ROS_INFO("callback_Clk_sync");
  return true;
}

unsigned argvParse(int &Argc, char **Argv, enum M &Mode){
  /* Check argc */
  if(Argc!=2){
    ROS_ERROR("Argc should be 2");
    return 1;
  }
  /* Check argv */
  std::string X = cspect::capitalize(Argv[1]);
  if(X.length()!=1){
    ROS_ERROR("Argv should be one letter, a b or c.");
    return 1;
  }
  if(     X[0] == 'A') Mode = A;
  else if(X[0] == 'B') Mode = B;
  else if(X[0] == 'C') Mode = C;
  else{
    ROS_ERROR("Argv wrong letter. Try a,b,c");
    return 1;
  }
  ROS_INFO("argvParse mode %c", modeChar(Mode));
  return 0;
}

char modeChar(enum M &Mode){
  switch(Mode){
  case A: return 'A';
  case B: return 'B';
  case C: return 'C';
  default: ROS_ERROR("modeChar function error"); return ' ';
  }
}

void open3DMap(){
  std::system(BASHCMD); /* new terminal tab run rviz node */
}  
void sendTo3DMap(double V[][3], ros::Publisher &Pub_Object_Markers, const unsigned &ArrSize){
  visualization_msgs::MarkerArray markers;
  bool unique[ArrSize];
  if(ArrSize < 1 || ArrSize > 10){
    ROS_ERROR("sendTo3DMap error. Set 1 <= ArrSize <= 10");
    return;
  }
  for(size_t ii = 0; ii!= ArrSize; ii++) unique[ii] = true;
  for(size_t ii = 0; ii!= ArrSize; ii++)
    for(size_t jj = ii+1; jj!= ArrSize; jj++)
      if(cspect::dist3D(V[ii],V[jj]) < EPSILON) unique[jj] = false;
  ROS_INFO("%d%d%d%d%d%d", (unsigned)unique[0], (unsigned)unique[1], (unsigned)unique[2],
	   (unsigned)unique[3], (unsigned)unique[4], (unsigned)unique[5]);
  for (size_t ii = 0; ii != ArrSize; ++ii) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; /* Adjust the frame ID as needed */
    marker.type = visualization_msgs::Marker::SPHERE; 
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = ii;
    marker.pose.position.x = V[ii][0]; 
    marker.pose.position.y = V[ii][1]; 
    marker.pose.position.z = V[ii][2]; 
    marker.color.r = 1.0 - (V[ii][2] / MAX_Z_VALUE); /* Z-coords color */
    marker.color.g = V[ii][2] / MAX_Z_VALUE;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    if(unique[ii]) marker.scale.x = marker.scale.y = marker.scale.z = MARKERSCALE;
    switch(ii){
    case 0: marker.ns = "DRONE"; break;
    case 1: marker.ns = "STATUS"; break;
    case 2: marker.ns = "BEACON1"; break;
    case 3: marker.ns = "BEACON2"; break;
    case 4: marker.ns = "BEACON3"; break;
    case 5: marker.ns = "BEACON4"; break;
    default: marker.ns = "UNKN"; break;
    }
    markers.markers.push_back(marker);
  }
  Pub_Object_Markers.publish(markers);
}
