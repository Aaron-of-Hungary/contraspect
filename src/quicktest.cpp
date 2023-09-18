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
#include <cmath>
#include <Eigen/Dense>

constexpr double PERIOD = 1.0;

void testDn_calc();

int main(int argc, char **argv) {

  /* Initialize node */
  ros::init(argc, argv, "quicktest");
  ros::NodeHandle nh;
  
  /* Initialize variables */

  
  /* Loop begin */
  ros::Rate loopRate(pow(PERIOD, -1.0));  // Control rate in Hz
  while (ros::ok()) {
    
    /* In-loop functions */
    testDn_calc();  
    
    /* Loop end */
    ros::spinOnce();
    loopRate.sleep();
    break;
  }
  return 0;
}

void testLoc_sim_calc(){
  double pos[2][3] = {{-.507704,.0,1.0},{-.0898603,.23488,1.0}};
  double bcn[4][3] =
    {{-1.0,0.0,0.0},{0.0,-1.0,0.0},{0.0,1.0,0.0},{1.0,0.0,0.0}};
  double dist[2][4] = {{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0}};
  double delay[2][4] = {{0.0,0.0,0.0,0.0},{0.0,0.0,0.0,0.0}};
  for(size_t ii=0; ii!=2; ii++)
    for(size_t jj=0; jj!=4; jj++){
      for(size_t kk=0; kk!=3; kk++)
	dist[ii][jj]+=pow(bcn[jj][kk]-pos[ii][kk],2.0);
      dist[ii][jj] = sqrt(dist[ii][jj]);
      delay[ii][jj] = dist[ii][jj] / 343;
      ROS_INFO("dist,delay[%lu][%lu]=%f,%f",ii,jj,dist[ii][jj],delay[ii][jj]);
    }
}

void testDn_calc(){
  double WV[4][3] = {{1,2,3},{4,5,6},{7,8,9},{10,11,12}};
  for(size_t wv = 0; wv!=4; wv++){
    ROS_INFO(" ");
    ROS_INFO("Weight Vector: %f,%f,%f,%f",WV[wv][0],WV[wv][1],WV[wv][2],WV[wv][3]);
    double bcn[4][3] = {{-1.0,.0,.0},{.0,-1.0,.0},
			{.0,1.0,.0} ,{1.0,.0,1.0}};  
    double delay[2][4] =
      {{ -.734911, -.746122, -.724211, -.756750},
       {-1.125641,-1.135376,-1.109148,-1.163239}};
    
    /* Numbers seen on image AKA read from Dn_calc */
    double dist_img[2][4] = {{-252.074,-255.920,-248.404,-259.565},
			     {-386.095,-389.434,-380.438,-398.991}};
    double Amtx_img[2][3][3]={{{4,0,0},{2,2,0},{2,-2,0}},
			      {{4,0,0},{2,2,0},{2,-2,0}}};
    double bvec_img[2][3]={{-3832.6,-1879.11,-5669.45},
			   {-10124.5,-7534.86,-14460.8}};
    double pos_img[2][3] = {{.0,.0,1.0},{.0,.0,1.0}};
    
    /* Numbers calculated here */
    double dist[2][4] = {{.0,.0,.0,.0},{.0,.0,.0,.0}};
    double Amtx[2][3][3]={{{.0,.0,.0},{.0,.0,.0},{.0,.0,.0}},
			  {{.0,.0,.0},{.0,.0,.0},{.0,.0,.0}}};
    double bvec[2][3]={{.0,.0,.0},{.0,.0,.0}};
    double pos[2][3] = {{.0,.0,.0},{.0,.0,.0}};
    
    /* Percentage difference between the two */
    double dist_diff[2][4] = {{.0,.0,.0,.0},{.0,.0,.0,.0}};
    double Amtx_diff[2][3][3]={{{.0,.0,.0},{.0,.0,.0},{.0,.0,.0}},
			       {{.0,.0,.0},{.0,.0,.0},{.0,.0,.0}}};
    double bvec_diff[2][3]={{.0,.0,.0},{.0,.0,.0}};
    double pos_diff[2][3] = {{.0,.0,.0},{.0,.0,.0}};
    
    for(size_t oo = 0; oo!=2; oo++){
      /* Calc dist from delay */
      for(size_t bb = 0; bb!=4; bb++){
	dist[oo][bb] = delay[oo][bb] * 343;
	dist_diff[oo][bb] = 100*abs(dist[oo][bb]-dist_img[oo][bb])/dist[oo][bb];
      }
      /* A MTX */
      for(size_t ii=0; ii!=3; ii++){
	for(size_t jj=0; jj!=3; jj++){
	  Amtx[oo][ii][jj] = 2.0*(bcn[3][jj]-bcn[ii][jj]);
	  if(abs(Amtx[oo][ii][jj]) < 0.0001) Amtx_diff[oo][ii][jj] = Amtx_img[oo][ii][jj];
	  else Amtx_diff[oo][ii][jj] = 100*abs(Amtx[oo][ii][jj]-Amtx_img[oo][ii][jj])/Amtx[oo][ii][jj];
	  /* b vec */
	  bvec[oo][ii] -= pow(bcn[ii][jj],2.0);
	  bvec[oo][ii] += pow(bcn[ 3][jj],2.0);
	}
	bvec[oo][ii] += pow(dist[oo][ii],2.0)-pow(dist[oo][3],2.0);
	bvec_diff[oo][ii] = 100*abs(bvec[oo][ii]-bvec_img[oo][ii])/bvec[oo][ii];
      }
      /* pos: x = ((AT*A)^-1 * AT) *b */
    }
    
    /* Transcribe to Eigen/Dense matrices, multiplying by weight vector */
    Eigen::MatrixXd mtxA1(3,3), mtxA2(3,3), vecb1(3,1), vecb2(3,1);
    for(size_t ii=0; ii!=3; ii++){
      vecb1(ii,0) = WV[wv][ii]*bvec[0][ii];
      vecb2(ii,0) = WV[wv][ii]*bvec[1][ii];
      for(size_t jj=0; jj!=3; jj++){
	mtxA1(ii,jj)=WV[wv][ii]*Amtx[0][ii][jj];
	mtxA2(ii,jj)=WV[wv][ii]*Amtx[1][ii][jj];
      }
    }
    
    /* Print Eigen/Dense matrices */
    ROS_INFO("vecb1T = %f\t%f\t%f",vecb1(0,0),vecb1(1,0),vecb1(2,0));
    ROS_INFO("vecb2T = %f\t%f\t%f",vecb2(0,0),vecb2(1,0),vecb2(2,0));
    for(size_t ii=0; ii!=3; ii++)
      ROS_INFO("mtxA1[%lu] = %f\t%f\t%f",ii,mtxA1(ii,0),mtxA1(ii,1),mtxA1(ii,2));
    for(size_t ii=0; ii!=3; ii++)
      ROS_INFO("mtxA2[%lu] = %f\t%f\t%f",ii,mtxA2(ii,0),mtxA2(ii,1),mtxA2(ii,2));
    
    /* Print transpose */
    ROS_INFO(" ");
    Eigen::MatrixXd mtxAT = (mtxA1.transpose()*mtxA1).inverse()*mtxA1.transpose();
    Eigen::MatrixXd XYZ = mtxAT * vecb1;
    for(size_t ii=0; ii!=3; ii++)
      ROS_INFO("mtxAT[%lu] = %f\t%f\t%f",ii,mtxAT(ii,0),mtxAT(ii,1),mtxAT(ii,2));
    ROS_INFO(" ");
    ROS_INFO("XYZ = %f\t%f\t%f",XYZ(0,0),XYZ(1,0),XYZ(2,0));
  }  
}
