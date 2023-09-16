/***
  Title: Dn_node
  Author: Aron Hajdu-Moharos
  Filename: contraspect/src/Dn_node.cpp
  Description: Simulation node of the Drone (Dn)
  Project: Contraspect Drone Location and Navigation System
***/

#include "ros/ros.h"
#include "contraspect/contraspect.h"
#include "contraspect_msgs/Triang.h"
#include "contraspect_msgs/Bcn_pos.h"
#include "contraspect_msgs/Clk_sync.h"
#include "contraspect_msgs/Status_map.h"
#include "contraspect_msgs/Dn_ctrl.h" 
#include "contraspect_msgs/CLK.h"
#include "std_msgs/String.h"
#include <sstream>
#include <cmath>
#include <Eigen/Dense>

constexpr char FILENAME[] = "/home/george/catkin_ws/src/contraspect/txt/Dn_node_info.txt",
      	       DSM[]      = "Dn_status_map";
constexpr double EPSILON  = 0.2; /* Max regular beacon message error. Must be NONZERO */

double map[cspect::BEACONS_NUM+1][3] =
  {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
double status[3] = {0.0,0.0,0.0};
float clk = 0.0;
double beacons_delay[cspect::BEACONS_NUM] = {0.0,0.0,0.0,0.0},
       beacons_dist [cspect::BEACONS_NUM] = {0.0,0.0,0.0,0.0};
/* [0] true only if all others true. default false */
bool triang_recvd[cspect::BEACONS_NUM+1] =  {false,false,false,false,false};
size_t cntr = 0;

void callback_Triang(const contraspect_msgs::Triang::ConstPtr& msg); 
bool callback_Clk_sync(    contraspect_msgs::Clk_sync::Request  &req,
		           contraspect_msgs::Clk_sync::Response &res); 
bool callback_Bcn_init_pos(contraspect_msgs::Bcn_pos ::Request  &req,
			   contraspect_msgs::Bcn_pos ::Response &res); 
bool callback_Dn_ctrl(     contraspect_msgs::Dn_ctrl ::Request  &req,
		           contraspect_msgs::Dn_ctrl ::Response &res);

unsigned argvParse(int &Argc, char **Argv, bool &Demo, std::stringstream &SS, double Map[][3]);
contraspect_msgs::Status_map setmsgDnStatusMap(double *Status, double Map[][3]);
void dronePosCalc(double *Beacons_Delay, double *Beacons_Dist, double Map[][3], ros::Publisher
		  &Pub_Dn_calc, const unsigned &N, std::stringstream &SS, const size_t &Cntr);


int main(int argc, char **argv){

  /* Print node info */  
  if(cspect::ROS_INFO_F(FILENAME)) ROS_WARN("Print node info failed.");
 
  /* Parse argv */  
  bool demo = false;
  std::stringstream ss;
  if (argvParse(argc, argv, demo, ss, map)){ 
    ROS_ERROR("Parse argv failed. Node shutdown.");
    return 1;    
  }
  /*ROS_INFO("Parse argv success");*/

  /* Initialize node */
  ss.str("Dn_node");
  cspect::ssEnd(ss);
  if(demo) ss << "_DEMO";
  std::string s = ss.str();
  ros::init(argc, argv, s.c_str());
  ros::NodeHandle n;

  /* Initialize pub sub variables */ 
  ss.str("");
  ss << "Triang";
  if(demo) ss << "_demo";
  s = ss.str();
  ros::Subscriber sub_Triang;
   sub_Triang = n.subscribe(s.c_str(),	 cspect::SUBRATE, callback_Triang);
  ros::Publisher pub_Dn_status_map, pub_CLK, pub_Dn_calc;
   pub_Dn_status_map = n.advertise<contraspect_msgs::Status_map>(DSM,       cspect::PUBRATE);
   pub_CLK	    = n.advertise<contraspect_msgs::CLK       >("CLK",     cspect::PUBRATE);
   pub_Dn_calc	    = n.advertise<        std_msgs::String    >("Dn_calc", cspect::PUBRATE);
  ros::ServiceServer ser_Clk_sync, ser_Dn_ctrl, ser_Bcn_init_pos;
   ser_Clk_sync          = n.advertiseService("Clk_sync_0"  , callback_Clk_sync    );
   ser_Bcn_init_pos      = n.advertiseService("Bcn_init_pos", callback_Bcn_init_pos);
   if(!demo) ser_Dn_ctrl = n.advertiseService("Dn_ctrl"     , callback_Dn_ctrl     );

  /* Loop begin */
  ros::Rate loopRate(pow(cspect::DPERIOD, -1.0));
  while(ros::ok()){

    /* Publish to CLK topic */  
    contraspect_msgs::CLK msg_CLK;
    msg_CLK.clk = clk;
    msg_CLK.bid = (uint8_t)0;
    pub_CLK.publish(msg_CLK);
    clk = clk + cspect::DPERIOD;
    /*ROS_INFO("Published to CLK topic");*/

    /* Publish to Dn_status_map topic - publish speed: 100Hz */
    if(!(cntr%cspect::DHZ_100)){
      contraspect_msgs::Status_map msg_Dn_status_map = setmsgDnStatusMap(status, map);
      for(size_t ii = 0; ii!=3; ii++)
	status[ii] = status[ii]-status[ii]*pow(0.5, 1.0/cspect::DPERIOD);
      pub_Dn_status_map.publish(msg_Dn_status_map);
    }
    /*ROS_INFO("Published to Dn_status_map topic");*/
    
    /* Calculate position and publish to Dn_calc topic - publish speed: 100Hz */ 
    triang_recvd[0] = true;
    for(size_t ii = 1; ii!=cspect::BEACONS_NUM+1; ii++)
      if(!triang_recvd[ii]) triang_recvd[0] = false;
    if(triang_recvd[0]){ /* triang msgs arrived from all 4 bcns */
      for(size_t ii = 1; ii!=cspect::BEACONS_NUM+1; ii++) triang_recvd[ii] = false;
      /* Publish speed encoded in this function: 100Hz*/
      dronePosCalc(beacons_delay, beacons_dist, map, pub_Dn_calc, cspect::BEACONS_NUM, ss, cntr);
      ROS_INFO("pub_Dn_calc");
    }   

    /* Loop end */
    ros::spinOnce();
    loopRate.sleep();
    cntr = cntr + 1;
  }
  return 0;
}

void callback_Triang(const contraspect_msgs::Triang::ConstPtr& msg){
  if(msg){
    /* Check if received data irregular */
    if(msg->bid<1||msg->bid>4){ /* bid irregular */
      ROS_ERROR("Received Triang message thrown, irregular BID");
      return;
    }
    beacons_delay[msg->bid-1] = clk-msg->timestamp;
    // ROS_INFO("callback_Triang msg.bid,msg.tmstp,this.clk,dly=%d,%f,%f,%f", msg->bid, msg->timestamp,clk,beacons_delay[msg->bid-1]);
  }
}

bool callback_Bcn_init_pos(contraspect_msgs::Bcn_pos ::Request  &req,
			   contraspect_msgs::Bcn_pos ::Response &res){
  /* Check req for irregularities */
  if(req.bid<1 || req.bid>4){ /* Bid incorrect */
    ROS_ERROR("Discarded Bcn_init_pos message with bid in incorrect range.");
    return false;
  }
  map[req.bid][0] = req.x;
  map[req.bid][1] = req.y;
  map[req.bid][2] = req.z;
  ROS_INFO("Bcn_init_pos->BID,X,Y,Z=%d,%f,%f,%f",
	   req.bid,map[req.bid][0],map[req.bid][1],map[req.bid][2]);
  res.bid = req.bid;
  return true;
}
bool callback_Dn_ctrl(contraspect_msgs::Dn_ctrl::Request  &req,
		      contraspect_msgs::Dn_ctrl::Response &res){  
  status[0] = status[0] + req.x;
  status[1] = status[1] + req.y;
  status[2] = status[2] + req.z;
  ROS_INFO("callback_Dn_ctrl");
  return true;
}  
bool callback_Clk_sync(contraspect_msgs::Clk_sync::Request  &req,
		       contraspect_msgs::Clk_sync::Response &res){
  if(req.all_syncd == (uint8_t)0){
    clk = 0.0;
    ROS_INFO("callback_Clk_sync not final");
  } else ROS_INFO("callback_Clk_sync final");
  return true;
} 

unsigned argvParse(int &Argc, char **Argv, bool &Demo, std::stringstream &SS, double Map[][3]){
  /* Checking argc */
  if(Argc!=4 && Argc!=5){
    ROS_ERROR("argc value incorrect.");
    return 1;
  }
  /* Parsing argv[1]-[3]: Dn init coords */
  try{
    Map[0][0] = std::stod(Argv[1]);
    Map[0][1] = std::stod(Argv[2]);
    Map[0][2] = std::stod(Argv[3]);
  }catch(const std::invalid_argument&){
    ROS_ERROR("argv[1]-[3] format incorrect. Try 0.5, 3.1415");
    return 1;
  }
  /* Parsing argv[4]: optional demo mode flag*/
  SS.str("");
  if(Argc!=5) return 0;
  cspect::ssEnd(SS);
  SS << cspect::capitalize(Argv[4]);
  if(SS.str() != "DEMO"){
    ROS_ERROR("argv[4] format incorrect. Try demo");
    return 1;
  }
  SS.str("");
  Demo = true;
  return 0;
}

contraspect_msgs::Status_map setmsgDnStatusMap(double *Status, double Map[][3]){
  contraspect_msgs::Status_map res;
  /* We assume beaconsNum == 4 */
  res.sx  = Status[0]; res.sy  = Status[1]; res.sz  = Status[2];
  res.dx  = Map[0][0]; res.dy  = Map[0][1]; res.dz  = Map[0][2];
  res.b1x = Map[1][0]; res.b1y = Map[1][1]; res.b1z = Map[1][2];
  res.b2x = Map[2][0]; res.b2y = Map[2][1]; res.b2z = Map[2][2];
  res.b3x = Map[3][0]; res.b3y = Map[3][1]; res.b3z = Map[3][2];
  res.b4x = Map[4][0]; res.b4y = Map[4][1]; res.b4z = Map[4][2];
  return res;
}

void dronePosCalc(double *Beacons_Delay, double *Beacons_Dist, double Map[][3], ros::Publisher
		  &Pub_Dn_calc, const unsigned &N, std::stringstream &SS, const size_t &Cntr){
  /* N: Beacons_Num */
  /* Calculate bcn dist from bcn delay */ 
  for(size_t ii = 0; ii!=3; ii++)
    Beacons_Dist[ii] = cspect::SPEEDOFSOUND * Beacons_Delay[ii];
  /*ROS_INFO("Calculate Bcn dist from delay success");*/ 
  /* Calculate dn pos from bcn dist.  d^2 = (xdn-xb)^2 + (ydn-yb)^2 + (zdn-zb)^2 */ 
  Eigen::MatrixXd mtxA(N-1, 3); // this might break it. if yes, initialize matrices properly
  Eigen::MatrixXd vecb(N-1, 1); // abandoning variable matrix size. fill up at init
  /* Filling up A matrix - works with variable Beacons_Num 
     Values: twice distance of final bcn from other bcns
     Dimensions: n-1x3, n=Beacons_Num */
  for(size_t ii = 0; ii!= N-1; ii++)
    for(size_t jj = 0; jj!=3; jj++)
      mtxA(ii,jj) = 2.0*(Map[N][jj]-Map[ii+1][jj]); /* Checked, correct. */
  /* Filling up b vector - works with variable Beacons_Num  
     Values: as you see 
     Dimensions: n-1x1 */
  for(size_t ii = 0; ii!= N-1; ii++){
    vecb(ii,0)=pow(Beacons_Dist[ii], 2.0)-pow(Beacons_Dist[N-1], 2.0);
    for(size_t jj = 0; jj!=3; jj++)
      vecb(ii,0) = vecb(ii,0) + pow(Map[N][jj], 2.0) - pow(Map[ii+1][jj], 2.0);
  }
  /* Matrix operations to calculate drone position. Operation: 
     x{3x1} = ((AT{3xn-1}xA{n-1x3})^-1{3x3} x AT{3xn-1}){3xn-1} x b{n-1x1}
     Legend: {dimension}, T: transpose, x: multiply */
  Eigen::MatrixXd x = ((mtxA.transpose()*mtxA).inverse()*(mtxA.transpose()))*vecb;
  /*ROS_INFO("Calculate Dn pos from Bcn dist success");*/
  /* Writing out the calculation. Like loc_sim. Publishing to Dn_calc. Publish speed: 100Hz */
  if(!(Cntr%cspect::DHZ_100)){
    SS.str("");
    cspect::ssEnd(SS);
    SS << "\nBeacon delays received via Triang topic."
       << "\n\tB1: " << std::to_string(Beacons_Delay[0])
       <<   "\tB2: " << std::to_string(Beacons_Delay[1])
       <<   "\tB3: " << std::to_string(Beacons_Delay[2])
       <<   "\tB4: " << std::to_string(Beacons_Delay[3]);
    SS << "\nBeacon distances calculated from delays."
       << "\n\t[d1,d2,d3,d4] = [" << Beacons_Dist[0] << ", " << Beacons_Dist[1] << ", "
       << Beacons_Dist[2] << ", " << Beacons_Dist[3] << "]";
    SS << "\nCalculation."
       << "\n\tSpeed of sound: c = " << cspect::SPEEDOFSOUND << " m/s"
       << "\n\tdist = delay * c";
    SS << "\nPosition of beacons received via Bcn_init_pos topic."
       << "\n\t[x1,y1,z1] = [" << Map[1][0] << ", " << Map[1][1] << ", " << Map[1][2] << "]"
       << "\n\t[x2,y2,z2] = [" << Map[2][0] << ", " << Map[2][1] << ", " << Map[2][2] << "]"
       << "\n\t[x3,y3,z3] = [" << Map[3][0] << ", " << Map[3][1] << ", " << Map[3][2] << "]"
       << "\n\t[x4,y4,z4] = [" << Map[4][0] << ", " << Map[4][1] << ", " << Map[4][2] << "]";
    SS << "\nPosition of drone calculated from beacon distances."
       << "\n\t[x, y, z ] = [" << Map[0][0] << ", " << Map[0][1] << ", " << Map[0][2] << "]";
    SS << "\nCreation of A matrix.";
    for(size_t ii = 0, jj = N/2-1; ii!=N-1; ii++){
      SS << "\n\t";
      if(ii==jj) SS << "A =";
      SS << "\t[ 2(x" << N << "-x" << ii+1 
	 <<   ") 2(y" << N << "-y" << ii+1 
	 <<   ") 2(z" << N << "-z" << ii+1 << ") ]";
      if(ii==jj) SS << "=";
      SS << "\t[";
      for(size_t kk = 0; kk!=3; kk++) SS << " " << mtxA(ii,kk);
      SS << " ]";
    }
    SS << "\nCreation of b vector.";
    for(size_t ii = 0, jj = N/2-1; ii!=N-1; ii++){
      SS << "\n\t";
      if(ii==jj) SS << "b =";
      SS << "\t[ d" << ii+1 << "^2-d" << N
	 << "^2-x"  << ii+1 << "^2-y" << ii+1 << "^2-z" << ii+1
	 << "^2+x"  << N    << "^2+y" << N    << "^2+z" << N    << " ]";
      if(ii==jj) SS << " =";
      SS << "\t[ " << vecb(ii,0) << " ]";
    }
    SS << "\nCalculate Dn pos."
       << "\n\tx = ((AT*A)^-1 * AT) * b"
       << "\n\nLegend: x: drone pos vector [x y z], T: mtx transpose";
    /* Publishing to Dn_calc*/ 
    std_msgs::String msg_Dn_calc;
    msg_Dn_calc.data = SS.str();
    Pub_Dn_calc.publish(msg_Dn_calc);
    /*ROS_INFO("Published to Dn_calc topic");*/
  }
}
