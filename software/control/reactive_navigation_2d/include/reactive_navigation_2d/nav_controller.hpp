#ifndef NAV_CONTROLLER_HPP
#define NAV_CONTROLLER_HPP


#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include <boost/function.hpp>
#include <map>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <Eigen/Core> 
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <string>
#include <urdf/model.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>

#include <math.h>
#include "reactive_navigation_2d/eigen_kdl_conversions.hpp"
#include "reactive_navigation_2d/angles.hpp"

#define DIST_ERROR 0.5
#define HEADING_ERROR 10*(M_PI/180)

namespace nav_control
{

   enum {STOPPED, RUNNING};

   double getTime_now()
   {
	struct timeval tv;
	gettimeofday (&tv,NULL);
	return (int64_t) tv.tv_sec*1000000+tv.tv_usec;
   };

 
// Example instantiation
//-------------------------------------
// int NrOfJoints  = kdl_chain.getNrOfJoints();
// nav_control::NavController<NrOfJoints> left_arm_controller(lcm,"LEFT_ARM_CMDS",kdl_chain);

  class NavController
  {

  public:
    // Ensure 128-bit alignment for Eigen
    // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //----------------constructor/destructor
    NavController(boost::shared_ptr<lcm::LCM> &lcm, const std::string &input_cmds_channel,
			 const std::string &robot_name,const urdf::Model &urdf_robot_model);
    ~NavController();

     void init();
     void update(const drc::robot_state_t &robot_state, const double &dt);
     void computePoseError(const Eigen::Affine3d &xact, const Eigen::Affine3d &xdes, Eigen::Matrix<double,6,1> &err);
     bool isRunning();

  private:
    //--------type defs
    //enum { JOINTS = 6 };

    typedef Eigen::Matrix<double, 6, 1> CartVector;
    typedef Eigen::Matrix<double,Eigen::Dynamic,1> VectorXq;
   //--------fields
  private:
    std::string _robot_name;
    urdf::Model _urdf_robot_model;
    std::vector<std::string> _chain_joint_names;

    
    uint controller_state;

    boost::shared_ptr<lcm::LCM> _lcm;    
    Eigen::Affine3d x, x_goal_;

    //--------Gains
    CartVector Kp_x, Kd_x;

    bool uninitialized_;

    int64_t latest_goal_timestamp_;

      //-------------message callback
    private:  
    void handleNavGoalMsg(const lcm::ReceiveBuffer* rbuf,
				  const std::string& chan, 
				  const drc::nav_goal_t* msg);

  bool debug_;

  }; //class NavController

  
  
  //==================constructor 

NavController::NavController(boost::shared_ptr<lcm::LCM> &lcm, 
	const std::string& input_cmds_channel,
	const std::string &robot_name,
	const urdf::Model &urdf_robot_model) : _lcm(lcm), _robot_name(robot_name), _urdf_robot_model(urdf_robot_model)
{
     std::cout << "\nSpawning a nav controller that listens to "<< input_cmds_channel << " channel." << std::endl;
   
 controller_state = STOPPED;
     //lcm ok?
    if(!_lcm->good())
      {
	std::cerr << "\nLCM Not Good: nav_controller" << std::endl;
	return;
      }

   // Set gains

  uninitialized_ = true; // flag to indicate that the controller is in uninitialized_ state

 // ---------------------- subscribe to goal commands 
_lcm->subscribe(input_cmds_channel, &nav_control::NavController::handleNavGoalMsg, this); 
   
}

//================== destructor 
NavController::~NavController() {

      drc::twist_t body_twist_cmd;
    body_twist_cmd.linear_velocity.x =0;
    body_twist_cmd.linear_velocity.y =0;
    body_twist_cmd.linear_velocity.z =0;
    body_twist_cmd.angular_velocity.x =0;
    body_twist_cmd.angular_velocity.y =0;
    body_twist_cmd.angular_velocity.z =0;
    _lcm->publish("NAV_CMDS", &body_twist_cmd);
  
  
}



//Controller needs initialization after the first robot state message is received.

void NavController::init() 
{
 debug_ = false;


}

//update() function is called from the robot state msg handler defined in controller_manager.

void NavController::update(const drc::robot_state_t &robot_state, const double &dt)
{


 if(uninitialized_)
 {
   this->init();  // only run for the first instance. 
   uninitialized_ = false;
 }

  drc::position_3d_t  body_position = robot_state.origin_position;
  drc::covariance_t  body_pos_cov = robot_state.origin_cov; // convert to uncertainty in goal
  // ======== Get current robot state
  transformLCMToEigen(body_position,x);



  CartVector x_err;
  computePoseError(x, x_goal_, x_err); //returns  x_goal_ - x
  double dx,dy,dtheta;
  dx = x_err[0];
  dy = x_err[1];
  dtheta = x_err[5]; // desired final heading error
  
  double desired_heading_2_goal = atan2(dy,dx);

  
  KDL::Frame kdl_x,kdl_x_goal;
  transformEigenToKDL(x,kdl_x);
  KDL::Rotation kdl_M = kdl_x.M;
  double roll,pitch,yaw;
  kdl_M.GetRPY(roll,pitch,yaw);
  double orientation_goal;
  transformEigenToKDL(x_goal_,kdl_x_goal);
  kdl_x_goal.M.GetRPY(roll,pitch,orientation_goal);
 

  // INSERT CONTROL ALGORITHM HERE
  double dtheta_goal = shortest_angular_distance(yaw,orientation_goal);
  double dheading = shortest_angular_distance(yaw,desired_heading_2_goal);
  double heading_gain = 1;
  double vel_gain = 0.0075;
  double d_goal  = sqrt(dx*dx + dy*dy); 

  //double lamda = std::min(0.25*(1/pow(d_goal,4)),1.0);
  double lamda = 0;
  if(d_goal<DIST_ERROR)
      lamda = 1;
  double w_dheading = (1-lamda)*dheading + lamda*dtheta_goal; 
  double wturn = std::min(1-pow((fabs(dheading)/M_PI),0.5),1.0);
   if (fabs(w_dheading) < (M_PI/8))
      wturn = 1;
  double desired_yaw_rate =  std::min(heading_gain*w_dheading,1.0);
  double desired_forward_speed = wturn*std::min(vel_gain*(pow(d_goal,2)/dt),1.0);
  if(d_goal<DIST_ERROR){
    desired_forward_speed = 0;
  }
  
  if(debug_){
  std::cout << "\n desired_forward_speed: " << desired_forward_speed<< std::endl;
  std::cout << " desired_yaw_rate: " << desired_yaw_rate<< std::endl;
  std::cout << " d_goal: " << d_goal << std::endl;

  std::cout << " X: " <<  x.translation()[0] << " " <<  x.translation()[1] << std::endl;
  std::cout << " goal: " <<  x_goal_.translation()[0] << " " <<  x_goal_.translation()[1] << std::endl;
  std::cout << " err: " << x_err.head<2>() << std::endl;
  std::cout << " dtheta: " << to_degrees(x_err[5]) << std::endl;
  std::cout << " dtheta_goal: " << to_degrees(dtheta_goal) << std::endl;
  std::cout << " dheading: " << to_degrees(dheading)<< std::endl;
  std::cout << " lamda: " << lamda << std::endl;
  std::cout << " w_dheading: " << to_degrees(w_dheading) << std::endl;
  }
    drc::twist_t body_twist_cmd;
   // body_twist_cmd.utime = getTime_now();
    //body_twist_cmd.robot_name = _robot_name;
    body_twist_cmd.linear_velocity.x =desired_forward_speed;
    body_twist_cmd.linear_velocity.y =0;
    body_twist_cmd.linear_velocity.z =0;
    body_twist_cmd.angular_velocity.x =0;
    body_twist_cmd.angular_velocity.y =0;
    body_twist_cmd.angular_velocity.z =desired_yaw_rate;

    _lcm->publish("NAV_CMDS", &body_twist_cmd); 

}//end update



 
void NavController::computePoseError(const Eigen::Affine3d &xact, const Eigen::Affine3d &xdes, Eigen::Matrix<double,6,1> &err)
{
// des - act
// Better error metric from KDL.	    
 KDL::Frame kdl_xdes,kdl_xact;
  transformEigenToKDL(xdes,kdl_xdes);
  transformEigenToKDL(xact,kdl_xact);

  KDL::Twist delta_pos;
  // delta pos required to transform xact to xdes
  delta_pos =KDL::diff(kdl_xact,kdl_xdes,1);
  transformKDLtwistToEigen(delta_pos,err);

// see KDL/src/frames.hpp  
//  
//  * KDL::diff() determines the rotation axis necessary to rotate the frame b1 to the same orientation as frame b2 and the vector
//  * necessary to translate the origin of b1 to the origin of b2, and stores the result in a Twist datastructure.   
// IMETHOD Twist diff(const Frame& F_a_b1,const Frame& F_a_b2,double dt=1);
//   IMETHOD Vector diff(const Rotation& R_a_b1,const Rotation& R_a_b2,double dt) {
// 	Rotation R_b1_b2(R_a_b1.Inverse()*R_a_b2);
// 	return R_a_b1 * R_b1_b2.GetRot() / dt;
// }


} // end computePoseError



  //=============message callbacks

void NavController::handleNavGoalMsg(const lcm::ReceiveBuffer* rbuf,
						 const std::string& chan, 
						 const drc::nav_goal_t* msg)						 
  { 

 /*    drc::nav_goal_t

	int64_t utime;
	string robot_name;
	position_3d_t goal_pos;
*/
 Eigen::Affine3d goal;
 transformLCMToEigen(msg->goal_pos,goal);
 
  CartVector x_err;
  computePoseError(x, goal, x_err);

  double dist_err_2d  = sqrt(x_err[0]*x_err[0]  + x_err[1]*x_err[1]);
 if((controller_state==RUNNING)&(dist_err_2d<DIST_ERROR)&(fabs(x_err[5])<HEADING_ERROR)){
//     controller_state=STOPPED;
//     uninitialized_ = true;

 }
 else if((controller_state==STOPPED)&(dist_err_2d>DIST_ERROR))
   controller_state=RUNNING; // as we just received a goal.
   
  if(controller_state==RUNNING)
  { 
    
        latest_goal_timestamp_ = msg->utime;
        x_goal_ = goal;

  }//if(controller_state==RUNNING)

	
 } // end handleNavGoalMsg
 

bool NavController::isRunning()
{
  return (controller_state==RUNNING);

}



  
} //end namespace 


#endif //NAV_CONTROLLER_HPP

