#ifndef NECK_OSCILLATOR_HPP
#define NECK_OSCILLATOR_HPP


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
#include "gaze_control/angles.hpp"

namespace gaze_control
{

   enum {STOPPED, RUNNING};

   double getTime_now()
   {
	  struct timeval tv;
	  gettimeofday (&tv,NULL);
	  return (int64_t) tv.tv_sec*1000000+tv.tv_usec;
   };

  class NeckOscillator
  {

  public:
    // Ensure 128-bit alignment for Eigen
    // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //----------------constructor/destructor
    NeckOscillator(boost::shared_ptr<lcm::LCM> &lcm, const std::string &robot_name);
    ~NeckOscillator();

     void init();
     drc::actuator_cmd_t update(const std::map<std::string, double> &jointpos_in, const std::map<std::string, double> &jointvel_in, const double &dt);

     bool isRunning();

   //--------fields
  private:
    std::string _robot_name;

    uint controller_state;

    boost::shared_ptr<lcm::LCM> _lcm;    

    bool uninitialized_;
   double neck_yaw_rate_desired;
   double neck_yaw_rate_prev;
   double neck_yaw_desired; 
   double pos_clamp, omega;

      //-------------message callback
    private:  
    /*void handleGazeGoalMsg(const lcm::ReceiveBuffer* rbuf,
				  const std::string& chan, 
				  const drc::gaze_goal_t* msg);*/

  bool debug_;

  }; //class NeckOscillator

  
  
  //==================constructor 

NeckOscillator::NeckOscillator(boost::shared_ptr<lcm::LCM> &lcm,
	const std::string &robot_name) : _lcm(lcm), _robot_name(robot_name)
{
 // std::cout << "\nSpawning a neck controller that listens to "<< input_cmds_channel << " channel." << std::endl;
 std::cout << "\nSpawning a neck oscillator" << std::endl;
 
 controller_state = RUNNING;
     //lcm ok?
    if(!_lcm->good())
      {
	std::cerr << "\nLCM Not Good: neck_controller" << std::endl;
	return;
      }


   // Set oscilation period
   omega = 0.2; //0.1 hz - one oscillations per sec.
   pos_clamp = 80*(M_PI/180);  // neck only has a range of +- 90 
   neck_yaw_desired  =pos_clamp;
   neck_yaw_rate_prev = 0;
   neck_yaw_rate_desired = 4*(pos_clamp)*omega; //osc/s to rads/s;  one full oscilation covers 4*80 deg traversal
   
  uninitialized_ = true; // flag to indicate that the controller is in uninitialized_ state

 // ---------------------- subscribe to goal commands 
 //_lcm->subscribe(input_cmds_channel, &neck_control::NeckOscillator::handleGazeGoalMsg, this); 
   
}

//================== destructor 
NeckOscillator::~NeckOscillator() {

}



//Controller needs initialization after the first robot state message is received.

void NeckOscillator::init() 
{
 debug_ = false;


}

//update() function is called from the robot state msg handler defined in controller_manager.

drc::actuator_cmd_t NeckOscillator::update(const std::map<std::string, double> &jointpos_in, const std::map<std::string, double> &jointvel_in, const double &dt)
{


 if(uninitialized_)
 {
   this->init();  // only run for the first instance. 
   uninitialized_ = false;
 }


  // ======== Get current neck state
   double neck_yaw_current =  jointpos_in.find("NeckYaw")->second; 
   double neck_yaw_rate_current = jointvel_in.find("NeckYaw")->second;
      double neck_pitch_current =  jointpos_in.find("NeckPitch")->second; 
   double neck_pitch_rate_current = jointvel_in.find("NeckPitch")->second;
   
   //========== desired ===========

   if(( neck_yaw_current > pos_clamp) & (neck_yaw_desired > 0) )
   {
        neck_yaw_rate_desired = -4*(pos_clamp)*omega;
        neck_yaw_desired = -pos_clamp; 
         
   }
   else if(( neck_yaw_current < -pos_clamp) & (neck_yaw_desired < 0) )
   {
         neck_yaw_rate_desired = 4*(pos_clamp)*omega;
         neck_yaw_desired = pos_clamp; 
    }
     

   
  // CONTROL ALGORITHM 

  double neck_pitch_desired = 0*(M_PI/180); //gaze command  
 double pos_error = shortest_angular_distance(neck_pitch_current,neck_pitch_desired );
 double K_p,K_d;
 K_p = 0.2;
 K_d = 0.05;
 double tau_pitch = K_p * pos_error - K_d * neck_pitch_rate_current;  // pos controller for pitch

double acc = (neck_yaw_rate_current - neck_yaw_rate_prev); // / std::min(dt,1e-3) 
double vel_error = neck_yaw_rate_desired - neck_yaw_rate_current;//desired yaw rate - neck_yaw_rate;
K_p = 0.1;
K_d = 0.01;
double tau_yaw = K_p * vel_error + K_d *acc; // velocity controller for yaw

neck_yaw_rate_prev =  neck_yaw_rate_current;
if(debug_){
 std::cout 
 << " yaw : " <<neck_yaw_current*(180/M_PI) 
 << " yaw rate: " <<neck_yaw_rate_current*(180/M_PI)  
 << " desired rate: " << neck_yaw_rate_desired*(180/M_PI) 
 << " error: " << vel_error*(180/M_PI)  
 << " acc: " << acc*(180/M_PI) 
 << " tau: " <<tau_yaw << std::endl;
 }
  
   drc::actuator_cmd_t torque_cmd;
   torque_cmd.utime = getTime_now();
   torque_cmd.robot_name = _robot_name;
   torque_cmd.num_actuators = 2;
   
	torque_cmd.actuator_name.push_back("NeckPitch");
  torque_cmd.actuator_effort.push_back(tau_pitch);
	torque_cmd.effort_duration.push_back(2*dt);// expires after 0.1 sec
	
	torque_cmd.actuator_name.push_back("NeckYaw");
  torque_cmd.actuator_effort.push_back(tau_yaw);
	torque_cmd.effort_duration.push_back(2*dt);// expires after 0.1 sec

  
  return torque_cmd;

}//end update





  //=============message callbacks

/*void NeckOscillator::handleGazeGoalMsg(const lcm::ReceiveBuffer* rbuf,
						 const std::string& chan, 
						 const drc::gaze_goal_t* msg)						 
  { 



	
 }  // end handleGazeGoalMsg */

 

bool NeckOscillator::isRunning()
{
  return (controller_state==RUNNING);

}



  
} //end namespace 


#endif //NECK_OSCILLATOR_HPP


