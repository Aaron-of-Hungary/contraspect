/***
  Title: Loc_sim
  Author: Aron Hajdu-Moharos
  Filename: contraspect_demo/src/Loc_sim.cpp
  Description: Simulation node of physical distance for demo
  Project: Contraspect Drone Location and Navigation System
***/

#include "ros/ros.h"
#include "contraspect/contraspect.h"
#include "contraspect_msgs/Triang.h" 
#include "contraspect_msgs/Bcn_pos.h" 
#include "contraspect_msgs/Status_map.h" 
#include "contraspect_msgs/Dn_ctrl.h" 
#include "std_msgs/String.h" 
#include <sstream>
#include <cmath>

constexpr char FILENAME[] = "/home/george/catkin_ws/src/contraspect_demo/txt/Loc_sim_info.txt",
      	       DNS[] 	  = "Dn_status_map";
constexpr double EPSILON    = 0.2     ,  /* Drone displace to trigger pos recalc. NONZERO! */
  	  	 SMALLVALUE = 0.000001;

double map[cspect::BEACONS_NUM+1][3] =
  {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
double status[3] = {0.0,0.0,0.0};
double beacons_dist   [cspect::BEACONS_NUM] = {0.0,0.0,0.0,0.0};
double beacons_delay  [cspect::BEACONS_NUM] = {0.0,0.0,0.0,0.0};
double recvd_timestamp[cspect::BEACONS_NUM] = {0.0,0.0,0.0,0.0};
double delay_counter  [cspect::BEACONS_NUM] = {0.0,0.0,0.0,0.0};
bool triang_recvd      [cspect::BEACONS_NUM] = {false,false,false,false}; /* Default false */
bool triang_sent       [cspect::BEACONS_NUM] = {false,false,false,false}; /* Default false */
bool init_pos_irreg[cspect::BEACONS_NUM]     = {true, true, true, true }; /* Default true  */

bool callback_Dn_ctrl(contraspect_msgs::Dn_ctrl::Request  &req,			     
		      contraspect_msgs::Dn_ctrl::Response &res);
void callback_Triang(		const contraspect_msgs::Triang    ::ConstPtr &msg);
void callback_Dn_status_map(    const contraspect_msgs::Status_map::ConstPtr &msg);

void delaysCalc(double *Beacons_Delay, double *Beacons_Dist, double Map[][3], ros::Publisher
		&Pub_Loc_sim_calc, const unsigned &Beacons_Num, const size_t &Cntr);

int main(int argc, char **argv){

  /* Print node info */
  if(cspect::ROS_INFO_F(FILENAME)) ROS_WARN("Print node info failed.");

  /* Initialize node and pub-sub variables */
  ros::init(argc, argv, "Loc_sim");
  ros::NodeHandle n;
  contraspect_msgs::Triang msg_Triang_demo;
  ros::Subscriber sub_Triang, sub_Dn_status_map;
   sub_Triang        = n.subscribe("Triang", cspect::SUBRATE, callback_Triang);
   sub_Dn_status_map = n.subscribe(DNS     , cspect::SUBRATE, callback_Dn_status_map);
  ros::Publisher pub_Triang_demo, pub_Loc_sim_calc;
   pub_Triang_demo = n.advertise<contraspect_msgs::Triang>("Triang_demo", cspect::PUBRATE);
   pub_Loc_sim_calc= n.advertise        <std_msgs::String>("Loc_sim_calc",cspect::PUBRATE);
  ros::ServiceServer ser_Dn_ctrl;
   ser_Dn_ctrl = n.advertiseService("Dn_ctrl", callback_Dn_ctrl);
  /* ROS_INFO("Initialize node success"); */

  /* Initializing additional variables */
  double moveSize = 0.0, DnPosArg = 0.0, distSumSq = 0.0;
  size_t cntr = 0;
  bool delay_recalc = false;
  
  /* Loop begin */
  ros::Rate loopRate(pow(cspect::PERIOD, -1.0));
  while(ros::ok()){
    
    /* Adjust drone pos */
    moveSize  = cspect::arg3D(status);
    DnPosArg = cspect::arg3D(map[0]);
    for(size_t ii = 0; ii!= cspect::BEACONS_NUM; ii++)
      distSumSq = distSumSq + pow(beacons_dist[ii], 2.0);
    if(!(cntr%cspect::P_10ms)){
      for(size_t ii = 0; ii!=3; ii++){
	map[0][ii] = map[0][ii] + status[ii] * (pow(0.5, 6.0));
	status[ii] = status[ii] - status[ii] * (pow(0.5, 6.0));
      }
      /*ROS_INFO("Statusx,y,z=%f,%f,%f",status[0],status[1],status[2]);*/
    }

    /* If Dn needs to move, flag to recalculate Triang forwarding delays */
    delay_recalc = false;
    if(moveSize / DnPosArg > EPSILON) delay_recalc = true; /* If Dn move call from DCS */
    else if(distSumSq < pow(EPSILON, 2.0)) delay_recalc = true; /* If dists not yet adjusted */
    /* Calculate Triang msg fwd delays, publish Loc_sim_calc. Speed: 1Hz*/
    if(delay_recalc && !(cntr%(cspect::P_10ms*100)))
      delaysCalc(beacons_delay, beacons_dist, map, pub_Loc_sim_calc, cspect::BEACONS_NUM, cntr);
    else if(!delay_recalc && !(cntr%(cspect::P_10ms*100))) ROS_INFO("No delays recalc.");

    /* Wait delay and publish to Triang_demo topic */
    for(size_t ii = 0; ii!= cspect::BEACONS_NUM; ii++)
      /* Wait to receive */
      if     (!triang_recvd[ii] && !triang_sent[ii]);
      /* Reset var2, begin wait */
      else if(!triang_recvd[ii] &&  triang_sent[ii]) triang_sent[ii] = false;
      /* Just sent, reset var1 */
      else if( triang_recvd[ii] &&  triang_sent[ii]) triang_recvd[ii] = false;
      /* Waiting to send */
      else if( triang_recvd[ii] && !triang_sent[ii]){
	/* Waiting to send, Add delay of size cspect::PERIOD */
	if(delay_counter[ii]<beacons_delay[ii])
	  delay_counter[ii] = delay_counter[ii] + cspect::PERIOD; 
	/* Waited delay, send */
	else{
	  msg_Triang_demo.timestamp = recvd_timestamp[ii];
	  msg_Triang_demo.bid = ii+1;
	  pub_Triang_demo.publish(msg_Triang_demo);
	  triang_sent[ii] = true;
	  ROS_INFO("Triang %lu sent. Delays: Calc,Actual: %f,%f",
		   ii+1, beacons_delay[ii], delay_counter[ii]);
	  delay_counter[ii] = 0.0;
	}	  
      }

    /* Loop end */
    ros::spinOnce();
    loopRate.sleep();
    cntr = cntr + 1;
  }
  return 0;
}

bool callback_Dn_ctrl(contraspect_msgs::Dn_ctrl::Request  &req,			     
		      contraspect_msgs::Dn_ctrl::Response &res){
  status[0] = status[0] + req.x;
  status[1] = status[1] + req.y;
  status[2] = status[2] + req.z;
  ROS_INFO("Loc_sim says: callback_Dn_ctrl");
  return true;
}

void callback_Triang(const contraspect_msgs::Triang::ConstPtr &msg){
  if(msg){
    if(triang_recvd[msg->bid-1] == false && triang_sent[msg->bid-1] == false){
      triang_recvd[msg->bid-1] = true;
      recvd_timestamp[msg->bid-1] = msg->timestamp;
      /*ROS_INFO("Triang msg received. Waiting to forward.");*/
    }else /*ROS_INFO("Triang msg discarded as previous not yet forwarded.");*/
      return;
  }
}
void callback_Dn_status_map(    const contraspect_msgs::Status_map::ConstPtr &msg){
  if(msg){
    double dn_pos_recvd[3] = {msg->dx, msg->dy, msg->dz};
    /* !!! IMPORTANT !!! Update Dn pos from Dn_status_map ONLY IF Dn moves significantly */
    /* IE. reinitialized */
    double distRecvdStord = cspect::dist3D(dn_pos_recvd, map[0]),
           argStatus = cspect::arg3D(status);
    /* Check if received Dn pos significantly different from stored */
    if(distRecvdStord/argStatus > 50.0*EPSILON && distRecvdStord > EPSILON){
      ROS_INFO("Loc_sim says: dn_pos_recvd x,y,z=%f,%f,%f\n\t\t\t\tdn_pos_stord x,y,z=%f,%f,%f\n\t\t\t\tdist,status=%f,%f",dn_pos_recvd[0],dn_pos_recvd[1],dn_pos_recvd[2],map[0][0],map[0][1],map[0][2],distRecvdStord,argStatus);
      map[0][0] = msg->dx;
      map[0][1] = msg->dy;
      map[0][2] = msg->dz;
      ROS_INFO("Loc_sim says: callback_Dn_status_map->Dn pos changed");
    }
    map[1][0] = msg->b1x; map[1][1] = msg->b1y; map[1][2] = msg->b1z; 
    map[2][0] = msg->b2x; map[2][1] = msg->b2y; map[2][2] = msg->b2z; 
    map[3][0] = msg->b3x; map[3][1] = msg->b3y; map[3][2] = msg->b3z; 
    map[4][0] = msg->b4x; map[4][1] = msg->b4y; map[4][2] = msg->b4z; 
    /*ROS_INFO("callback_Dn_status_map");*/
  }
}

void delaysCalc(double *Beacons_Delay, double *Beacons_Dist, double Map[][3], ros::Publisher
		&Pub_Loc_sim_calc, const unsigned &Beacons_Num, const size_t &Cntr){
  /* Calculate distance from map then delay */
  double dist[3] = {0.0,0.0,0.0};
  for(size_t ii = 0; ii!=Beacons_Num; ii++){
    Beacons_Dist[ii] = 0.0;
    for(size_t jj = 0; jj!=3; jj++){
      dist[jj] = fabs(map[0][jj]-map[ii+1][jj]);
      Beacons_Dist[ii] = Beacons_Dist[ii] + pow(dist[jj], 2.0);
    }
    Beacons_Dist[ii] = pow(Beacons_Dist[ii], 0.5);
    Beacons_Delay[ii] = Beacons_Dist[ii] / cspect::SPEEDOFSOUND;
  }
  /* Publish calculation to Loc_sim_calc topic. */
  std::stringstream ss;
  ss << "Position of drone."
     << "\n\t[x, y, z ] = [" << Map[0][0] << ", " << Map[0][1] << ", " << Map[0][2] << "]";
  ss << "\nPosition of beacons."
     << "\n\t[x1,y1,z1] = [" << Map[1][0] << ", " << Map[1][1] << ", " << Map[1][2] << "]"
     << "\n\t[x2,y2,z2] = [" << Map[2][0] << ", " << Map[2][1] << ", " << Map[2][2] << "]"
     << "\n\t[x3,y3,z3] = [" << Map[3][0] << ", " << Map[3][1] << ", " << Map[3][2] << "]"
     << "\n\t[x4,y4,z4] = [" << Map[4][0] << ", " << Map[4][1] << ", " << Map[4][2] << "]";
  ss << "\nBeacon distances."
     << "\n\t[d1,d2,d3,d4] = [" << Beacons_Dist[0] << ", " << Beacons_Dist[1] << ", "
     << Beacons_Dist[2] << ", " << Beacons_Dist[3] << "]";
  ss << "\nCalculation."
     << "\n\tdi = sqrt( (xi-x)^2 + (yi-y)^2 + (zi-z)^2 )";
  ss << "\nBeacon delays to be added."
     << "\n\tB1: " << Beacons_Delay[0] << "\tB2: " << Beacons_Delay[1]
     <<   "\tB3: " << Beacons_Delay[2] << "\tB4: " << Beacons_Delay[3];
  ss << "\nCalculation."
     << "\n\tSpeed of sound: c = " << cspect::SPEEDOFSOUND << " m/s"
     << "\n\tdelay = dist / c";
  std_msgs::String msg_Loc_sim_calc;
  msg_Loc_sim_calc.data = ss.str();
  Pub_Loc_sim_calc.publish(msg_Loc_sim_calc);
  ROS_INFO("pub_Loc_sim_calc");
}
