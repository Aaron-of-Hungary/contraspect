/***
  Title: Beacon_node
  Author: Aron Hajdu-Moharos
  Filename: contraspect/src/Beacon_node.cpp
  Description: Simulation node of a homing beacon (bcn)
  Project: Contraspect Drone Location and Navigation System
***/

#include "ros/ros.h"
#include "contraspect/contraspect.h"
#include "contraspect_msgs/Clk_sync.h"
#include "contraspect_msgs/Triang.h"
#include "contraspect_msgs/CLK.h"
#include "contraspect_msgs/Bcn_pos.h"
#include <sstream>

constexpr char   FILENAME[] = "/home/george/catkin_ws/src/contraspect/txt/Beacon_node_info.txt";

float clk = 0.0;
double pos[3] = {0.0,0.0,0.0};
uint8_t bid = 0;
bool clk_syncd = false;

bool callback_Clk_sync(contraspect_msgs::Clk_sync::Request  &req,
		       contraspect_msgs::Clk_sync::Response &res);
unsigned argvParse(int Argc, char **Argv, double *Pos, uint8_t &Bid);

int main(int argc, char **argv){

  /* Print node info */
  if(cspect::ROS_INFO_F(FILENAME)) ROS_WARN("Print node info failed.");

  /* Parse argv */
  if(argvParse(argc, argv, pos, bid)){
    ROS_ERROR("Parse argv failed. Node shutdown.");
    return 1;
  }
  /*ROS_INFO("Parse argv success");*/

  /* Initialize node and pub sub variables */
  std::stringstream ss;
  ss << "Beacon_node_" << (unsigned)bid;
  std::string s = ss.str();
  ros::init(argc, argv, s.c_str());
  ros::NodeHandle n;
  ros::Publisher pub_Triang =n.advertise<contraspect_msgs::Triang>("Triang",cspect::PUBRATE);
  ros::Publisher pub_CLK    =n.advertise<contraspect_msgs::CLK   >("CLK"   ,cspect::PUBRATE);
  ros::ServiceClient cli_Bcn_init_pos=n.serviceClient<contraspect_msgs::Bcn_pos>("Bcn_init_pos");
  ros::ServiceServer ser_Clk_sync;
   ss.str("");
   ss << "Clk_sync_" << (unsigned)bid;
   s = ss.str();
   ser_Clk_sync = n.advertiseService(s.c_str(), callback_Clk_sync);
  /*ROS_INFO("Initialize node and pub sub variables success");*/

  /* Initialize additional variables */
  contraspect_msgs::Triang msg_Triang;
  contraspect_msgs::CLK msg_CLK;
  contraspect_msgs::Bcn_pos srv_Bcn_init_pos;
  srv_Bcn_init_pos.request.x   = pos[0];
  srv_Bcn_init_pos.request.y   = pos[1];
  srv_Bcn_init_pos.request.z   = pos[2];
  srv_Bcn_init_pos.request.bid = bid;
  srv_Bcn_init_pos.response.bid = (uint8_t)(cspect::BEACONS_NUM*2);
  bool init_pos_done = false;
  unsigned cntr = 0;
  
  /*Loop begin */
  ros::Rate loopRate(pow(cspect::BPERIOD, -1.0));
  while(ros::ok()){

    /* Publish to Triang topic */
    /* publish for 1ms, then pause for 9ms */
    if(clk_syncd && cspect::lastdigit_msec(clk) == 0){
      msg_Triang.timestamp = clk;
      msg_Triang.bid = bid;
      pub_Triang.publish(msg_Triang);
      ROS_INFO("pub Triang tmstp,bid=%f,%d",msg_Triang.timestamp,msg_Triang.bid);
    }
      
    /* Publish to CLK topic */
    msg_CLK.clk = clk;
    msg_CLK.bid = bid;
    pub_CLK.publish(msg_CLK);
    /*ROS_INFO("CLK pub: {clk,bid}={%f,%d}",msg_CLK.clk,msg_CLK.bid);*/

    /* Call Bcn_init_pos service */
    if(!init_pos_done && !((cntr/10)%10))
      if(cli_Bcn_init_pos.call(srv_Bcn_init_pos)){
	if(srv_Bcn_init_pos.response.bid == (uint8_t)bid){
	  init_pos_done = true;
	  ROS_INFO("call srv_Bcn_init_pos %d: x,y,z,rqbid,rpbid = %f,%f,%f,%d,%d",
		       cntr, pos[0], pos[1], pos[2],
		       srv_Bcn_init_pos.request.bid,srv_Bcn_init_pos.response.bid);
	}else ROS_WARN("WRONG BID call srv_Bcn_init_pos %d: x,y,z,rqbid,rpbid = %f,%f,%f,%d,%d",
		       cntr, pos[0], pos[1], pos[2],
		       srv_Bcn_init_pos.request.bid,srv_Bcn_init_pos.response.bid);
      }else ROS_ERROR("FAILED call srv_Bcn_init_pos %d: x,y,z,rqbid,rpbid = %f,%f,%f,%d,%d",
		       cntr, pos[0], pos[1], pos[2],
		       srv_Bcn_init_pos.request.bid,srv_Bcn_init_pos.response.bid);
    cntr = cntr + 1;

    /* Loop end */
    clk = clk + cspect::BPERIOD;
    ros::spinOnce();
    loopRate.sleep();
  }

  /* End of code */
  return 0;
}

bool callback_Clk_sync(contraspect_msgs::Clk_sync::Request  &req,
		       contraspect_msgs::Clk_sync::Response &res){
  if(req.all_syncd>0){
    clk_syncd = true;
    ROS_INFO("callback_Clk_sync final");
  }else{
    clk = 0.0;
    ROS_INFO("callback_Clk_sync not final");
  }
  return true;
}

unsigned argvParse(int Argc, char **Argv, double *Pos, uint8_t &Bid){
  /* Check argc */
  if(Argc!=5){
    ROS_ERROR("Argc should be 5");
    return 1;
  }
  /* Parse argv[1]: Beacon ID (bid) */
  std::string s = Argv[1];
  if(s.length()==1 && s[0]>='1' && s[0]<='4') /* Format correct */
    Bid = (uint8_t)(s[0]-'0');
  else{ /* Format incorrect */
    ROS_ERROR("Argv[1] format incorrect. Try 1, 2, 3, 4");
    return 1;
  }
  /*ROS_INFO("Set beacon ID to %d from argv[1]", Bid);*/
  /* Parse argv [2]-[4]: Beacon init coords */
  try{
    Pos[0] = std::stod(Argv[2]);
    Pos[1] = std::stod(Argv[3]);
    Pos[2] = std::stod(Argv[4]);
  }catch(const std::invalid_argument&){
    ROS_ERROR("argv[2]-[4] format incorrect. Try 0.5, 3.1415");
    return 1;
  }
  ROS_INFO("argvParse: x,y,z,bid=%f,%f,%f,%d",Pos[0],Pos[1],Pos[2],Bid);
  return 0;  
}