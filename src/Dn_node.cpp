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
#include <vector>

constexpr char FILENAME[] = "/home/george/catkin_ws/src/contraspect/txt/Dn_node_info.txt";
constexpr double EPSILON  = 0.2, /* Max regular beacon message error. Must be NONZERO */
  PERIOD = cspect::PERIOD;       /* Do not meddle with this */
constexpr unsigned PERIOD_DN_POS_CALC = cspect::P_10ms*10, /* droneposcalc, pub_Dn_calc, 10Hz */
  DELAYS_VECTOR_SIZE = 100, /* Number of delays per bcn whose average calculates Drone pos */
  BN = cspect::BEACONS_NUM;

/* Init global vars */
float clk = .0;
std::array<double, BN> beacons_delay = {.0,.0,.0,.0};
double map[BN+1][3] = {{.0,.0,.0},{.0,.0,.0},{.0,.0,.0},{.0,.0,.0},{.0,.0,.0}},
  beacons_dist [BN] = {.0,.0,.0,.0},
  status       [3] = {.0,.0,.0},
  base_delay   [7] = {.0,.0,.0,.0,1.0,.0,.0};/* Min, Max, Mean, Stdev, Absmin, Absmax, AbsMean */
/* [0] true only if all others true. default false: */
bool triang_recvd[BN+1] =  {false,false,false,false,false};
size_t cntr = 0, triang_cntr = 0;
std::vector<std::array<double,BN>> delays_vector;

/* Declare */
void callback_Triang(const  contraspect_msgs::Triang    ::ConstPtr &msg); 
bool callback_Bcn_init_pos (contraspect_msgs::Bcn_pos   ::Request  &req,
			    contraspect_msgs::Bcn_pos   ::Response &res); 
bool callback_Dn_ctrl      (contraspect_msgs::Dn_ctrl   ::Request  &req,
		            contraspect_msgs::Dn_ctrl   ::Response &res);
bool callback_Dn_status_map(contraspect_msgs::Status_map::Request  &req,
			    contraspect_msgs::Status_map::Response &res);
			    
unsigned argvParse(int &Argc, char **Argv, bool &Demo, std::stringstream &SS,
		   double Map[BN+1][3]);
unsigned dronePosCalc(const unsigned& Delays_vector_size,double *Delays_avgs,double Map[BN+1][3],
		      std::vector<std::array<double,BN>> &Delays_vector, double *Dists,
		      ros::Publisher &Pub_Dn_calc, const double W[BN], std::stringstream &SS);

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
  ros::Publisher pub_CLK, pub_Dn_calc;
  //   pub_CLK	    = n.advertise<contraspect_msgs::CLK       >("CLK",     cspect::PUBRATE);
   pub_Dn_calc	    = n.advertise<        std_msgs::String    >("Dn_calc", cspect::PUBRATE);
  ros::ServiceServer ser_Dn_ctrl, ser_Bcn_init_pos, ser_Dn_status_map;
   ser_Dn_status_map     = n.advertiseService("Dn_status_map", callback_Dn_status_map);
   ser_Bcn_init_pos      = n.advertiseService("Bcn_init_pos" , callback_Bcn_init_pos );
   if(!demo) ser_Dn_ctrl = n.advertiseService("Dn_ctrl"      , callback_Dn_ctrl      );
  ros::ServiceClient cli_Clk_sync;
   cli_Clk_sync = n.serviceClient<contraspect_msgs::Clk_sync>("Clk_sync");

  /* Initialize other variables */
  contraspect_msgs::Clk_sync srv_Clk_sync;
  char c;
  double delays_avgs[BN];

  /* Loop begin */
  ros::Rate loopRate(pow(PERIOD, -1.0));
  while(ros::ok()){

    // Temporarily not publishing due to potential strain on roscore due to high freq
    //    /* Publish to CLK topic */
    //    if(cspect::lastdigit_msec(clk)==0 && !(cntr%(BN+1))){
    //      contraspect_msgs::CLK msg_CLK;
    //      msg_CLK.clk = clk;
    //      msg_CLK.bid = (uint8_t)0;
    //      pub_CLK.publish(msg_CLK);
    //      /*ROS_INFO("Published to CLK topic");*/
    //    }
    
    /* Call CLK_sync srv req */
    if(!(cntr%(unsigned)(1.0/PERIOD/cspect::CLK_SYNC_FREQ+1.0))){
      srv_Clk_sync.request.clk = clk;
      if(cli_Clk_sync.call(srv_Clk_sync)){
	clk = clk + srv_Clk_sync.response.adjust;
	srv_Clk_sync.response.adjust<0.0 ? c='-' :  c='+';
	/*ROS_INFO("Dn Clk_sync adjust %c%f",c,srv_Clk_sync.response.adjust);*/
      }
      else ROS_ERROR("Dn Clk_sync adjust fail");
    }

    /* Triang msgs arrived from all 4 bcns */
    if(triang_recvd[0]){
      /* Reset triang_recvd array */
      for(size_t ii = 0; ii!=BN+1; ii++) triang_recvd[ii] = false;
      /* Add received delays data to delays_vector */
      delays_vector.push_back(beacons_delay);
      /* Cycle delays_vector, keep at desired size by deleting oldest value */
      if(delays_vector.size()>DELAYS_VECTOR_SIZE) delays_vector.erase(delays_vector.begin());
      /* Calculate position and publish to Dn_calc topic */
      if(!(cntr%PERIOD_DN_POS_CALC)){
	/* ROS_INFO("triang_recvd:%d%d%d%d%d",(int)triang_recvd[0],(int)triang_recvd[1],
	   (int)triang_recvd[2],(int)triang_recvd[3],(int)triang_recvd[4]);*/
	// Add avgs array
	if(dronePosCalc(DELAYS_VECTOR_SIZE,delays_avgs,map,delays_vector,beacons_dist,
			pub_Dn_calc,cspect::BEACONS_WEIGHT,ss))
	  ROS_ERROR("dronePosCalc ERROR");
      }
    }

    /* Display base delay data */
    if(!(cntr%(cspect::P_10ms*100)))
      ROS_INFO
	("Base delay %lu\tmin,max,mean,stdev=%f,%f,%f,%f || absmin,absmax,absmean=%f,%f,%f",
	 triang_cntr  ,base_delay[0],base_delay[1],base_delay[2],
	 base_delay[3],base_delay[4],base_delay[5],base_delay[6]);

    /* Shift clk */
    if(clk>2048.0-PERIOD) clk = clk-2048.0+PERIOD; else clk = clk + PERIOD;
    /* Loop end */
    cntr = cntr + 1;
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}

void callback_Triang(const contraspect_msgs::Triang::ConstPtr& msg){
  if(msg){
    /* Check if received data irregular */
    if(msg->bid<1||msg->bid>4){ /* bid irregular */
      ROS_ERROR("Received Triang message thrown, irregular BID %d",(int)msg->bid);
      return;
    }
    if(triang_recvd[msg->bid]); /*skip*/
    /* Only start logging after first 8 triang messages received for accuracy */
    else if(triang_cntr>2*BN){
      /* Mark triang received */
      triang_recvd[msg->bid] = true;
      /* Set beacons delay, mark triang received */
      beacons_delay[msg->bid-1] = clk-msg->timestamp;
      /* Check if all triang msgs received */
      triang_recvd[0] = true;
      for(size_t ii = 1; ii!=BN+1; ii++)
	if(!triang_recvd[ii]) triang_recvd[0] = false;
      /* Ros Info regarding set delay */
      /*ROS_INFO("callback_Triang. delay[%d]=Dn.clk-msg.tmstp=%f-%f=%f", 
	       msg->bid,clk, msg->timestamp,beacons_delay[msg->bid-1]);*/

      /* Analytics: Measure base delay (MIN,MAX,MEAN,STDEV,ABSMIN,ABSMAX,ABSMEAN) */
      /* set min */
      if(beacons_delay[msg->bid-1]<base_delay[0])
	base_delay[0] = beacons_delay[msg->bid-1];
      /* set max */
      else if(beacons_delay[msg->bid-1]>base_delay[1])
	base_delay[1] = beacons_delay[msg->bid-1];      
      /* set mean */
      base_delay[2]=(base_delay[2]*triang_cntr+beacons_delay[msg->bid-1])/(triang_cntr+1);
      /* set standard deviation */
      if(triang_cntr>1)
	base_delay[3]=sqrt((pow(base_delay[3],2.0)*(triang_cntr-1)+
			    pow(beacons_delay[msg->bid-1]-base_delay[2],2.0))/triang_cntr);
      /* set absmin */
      if(triang_cntr>4 && fabs(beacons_delay[msg->bid-1])<fabs(base_delay[4]))
	base_delay[4] = beacons_delay[msg->bid-1];
      /* set absmax */
      else if(fabs(beacons_delay[msg->bid-1])>fabs(base_delay[5]))
	base_delay[5] = beacons_delay[msg->bid-1];
      /* set absmean */
      base_delay[6]=(base_delay[6]*triang_cntr+fabs(beacons_delay[msg->bid-1]))/(triang_cntr+1);
      /* increment triang counter */
      triang_cntr = triang_cntr+1;
    }else triang_cntr = triang_cntr+1;
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

unsigned argvParse(int &Argc, char **Argv, bool &Demo,
		   std::stringstream &SS, double Map[BN+1][3]){
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

bool callback_Dn_status_map(contraspect_msgs::Status_map::Request  &req,
			    contraspect_msgs::Status_map::Response &res){
  /* We assume beaconsNum == 4 */
  res.sx  = status[0]; res.sy  = status[1]; res.sz  = status[2];
  res.dx  = map[0][0]; res.dy  = map[0][1]; res.dz  = map[0][2];
  res.b1x = map[1][0]; res.b1y = map[1][1]; res.b1z = map[1][2];
  res.b2x = map[2][0]; res.b2y = map[2][1]; res.b2z = map[2][2];
  res.b3x = map[3][0]; res.b3y = map[3][1]; res.b3z = map[3][2];
  res.b4x = map[4][0]; res.b4y = map[4][1]; res.b4z = map[4][2];
  ROS_INFO("callback_Dn_status_map");
  return true;
}

unsigned dronePosCalc(const unsigned& Delays_vector_size,double *Delays_avgs,double Map[BN+1][3],
		      std::vector<std::array<double,BN>> &Delays_vector, double *Dists,
		      ros::Publisher &Pub_Dn_calc, const double W[BN], std::stringstream &SS){
  /* BN: Beacons_Num, W: Beacon_Weight */
  /* Check delays vector size */
  if(Delays_vector_size!=Delays_vector.size()) {
    ROS_ERROR("dronePosCalc error, delays vector wrong size.");
    return 1;
  }
  /* Initializing delays_vector. */
  /* set new delays as the average of previous received delays who number Delays_vector_size */
  for(size_t ii = 0; ii!=BN; ii++) {
    Delays_avgs[ii]=.0;
    for(size_t jj = 0; jj!=Delays_vector.size(); jj++)
      Delays_avgs[ii] = Delays_avgs[ii] + Delays_vector[jj][ii];
    Delays_avgs[ii] = Delays_avgs[ii] / Delays_vector.size();
  }
  /* Calculate bcn dist from bcn delay */ 
  for(size_t ii = 0; ii!=BN; ii++) Dists[ii] = cspect::SPEEDOFSOUND * Delays_avgs[ii];
  /*ROS_INFO("Calculate Bcn dist from delay success");*/ 
  /* Calculate dn pos from bcn dist.  d^2 = (xdn-xb)^2 + (ydn-yb)^2 + (zdn-zb)^2 */ 
  Eigen::MatrixXd mtxA(BN-1, 3); // this might break it. if yes, initialize matrices properly
  Eigen::MatrixXd vecb(BN-1, 1); // abandoning variable matrix size. fill up at init
  /* Filling up A matrix - works with variable Beacons_Num 
     Values: twice distance of final bcn from other bcns
     Dimensions: n-1x3, n=Beacons_Num */
  /* Checked, correct. */
  for(size_t ii = 0; ii!= BN-1; ii++)
    for(size_t jj = 0; jj!=3; jj++) mtxA(ii,jj) = W[ii]/W[BN-1]*2.0*(Map[BN][jj]-Map[ii+1][jj]);
  /* Filling up b vector - works with variable Beacons_Num  
     Values: as you see 
     Dimensions: n-1x1 */
  for(size_t ii = 0; ii!= BN-1; ii++){
    vecb(ii,0)=pow(Dists[ii], 2.0)-pow(Dists[BN-1], 2.0);
    for(size_t jj = 0; jj!=3; jj++)
      vecb(ii,0) = vecb(ii,0) + pow(Map[BN][jj], 2.0) - pow(Map[ii+1][jj], 2.0);
    vecb(ii,0) = W[ii]/W[BN-1]*vecb(ii,0);
  }
  /* Matrix operations to calculate drone position. Operation: 
     x{3x1} = ((AT{3xn-1}xA{n-1x3})^-1{3x3} x AT{3xn-1}){3xn-1} x b{n-1x1}
     Legend: {dimension}, T: transpose, x: multiply */
  Eigen::MatrixXd X = ((mtxA.transpose()*mtxA).inverse()*(mtxA.transpose()))*vecb;
  for(size_t ii=0; ii!=3; ii++)
    if(std::isnan(X(ii,0))){
      ROS_ERROR("dronePosCalc fatal error. Likely reason: beacon positions all in one plane.");
      return 1;
      /* Setting Drone position values */
    } /*else Map[0][ii] = X(ii,0);*/
  /*ROS_INFO("Calculate Dn pos from Bcn dist success");*/
  /* Writing out the calculation, publishing to Dn_calc. Like loc_sim. */
  SS.str("");
  cspect::ssEnd(SS);
  SS << "\nBeacon delays received via Triang topic."
     << "\n\tB1: " << std::to_string(Delays_avgs[0])
     <<   "\tB2: " << std::to_string(Delays_avgs[1])
     <<   "\tB3: " << std::to_string(Delays_avgs[2])
     <<   "\tB4: " << std::to_string(Delays_avgs[3]);
  SS << "\nBeacon distances calculated from delays."
     << "\n\t[d1,d2,d3,d4] = ["
     << Dists[0] << ", " << Dists[1] << ", " << Dists[2] << ", " << Dists[3] << "]";
  SS << "\nCalculation."
     << "\n\tSpeed of sound: c = " << cspect::SPEEDOFSOUND << " m/s"
     << "\n\tdist = delay * c";
  SS << "\nPosition of beacons received via Bcn_init_pos topic."
     << "\n\t[x1,y1,z1] = [" << Map[1][0] << ", " << Map[1][1] << ", " << Map[1][2] << "]"
     << "\n\t[x2,y2,z2] = [" << Map[2][0] << ", " << Map[2][1] << ", " << Map[2][2] << "]"
     << "\n\t[x3,y3,z3] = [" << Map[3][0] << ", " << Map[3][1] << ", " << Map[3][2] << "]"
     << "\n\t[x4,y4,z4] = [" << Map[4][0] << ", " << Map[4][1] << ", " << Map[4][2] << "]";
  SS << "\nPosition of drone calculated from beacon distances."
     << "\n\t[x, y, z ] = [" << X(0,0) << ", " << X(1,0) << ", " << X(2,0) << "]";
  SS << "\nCreation of A matrix.";
  for(size_t ii = 0, jj = BN/2-1; ii!=BN-1; ii++){
    SS << "\n\t";
    if(ii==jj) SS << "A =";
    SS << "\t[ 2(x" << BN << "-x" << ii+1 
       <<   ") 2(y" << BN << "-y" << ii+1 
       <<   ") 2(z" << BN << "-z" << ii+1 << ") ]";
    if(ii==jj) SS << "=";
    SS << "\t[";
    for(size_t kk = 0; kk!=3; kk++) SS << "\t" << mtxA(ii,kk);
    SS << "\t]";
  }
  SS << "\nCreation of b vector.";
  for(size_t ii = 0, jj = BN/2-1; ii!=BN-1; ii++){
    SS << "\n\t";
    if(ii==jj) SS << "b =";
    SS << "\t[ d" << ii+1 << "^2-d" << BN
       << "^2-x"  << ii+1 << "^2-y" << ii+1 << "^2-z" << ii+1
       << "^2+x"  << BN    << "^2+y" << BN    << "^2+z" << BN    << "^2 ]";
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
  /*ROS_INFO("pub_Dn_calc");*/
  return 0;
}
