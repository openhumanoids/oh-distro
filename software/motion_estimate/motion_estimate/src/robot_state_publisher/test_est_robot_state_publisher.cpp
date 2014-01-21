// Test program that listens to robot urdf and publishes its state on EST_ROBOT_STATE with zero joint angles

#include <iostream>
#include <stdint.h> 
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <algorithm>

#include <urdf/model.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/robot_urdf_t.hpp"

using namespace std;

namespace test_est_robot_state_publisher {
  
class RobotModel {
 public:
   lcm::LCM lcm;
   std::string urdf_xml_string; 
   std::vector<std::string> joint_names_;
 };

void onMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg, RobotModel* robot) {
  // Received robot urdf string. Store it internally and get all available joints.

   robot->urdf_xml_string = msg->urdf_xml_string;
  std::cout<<"Received urdf_xml_string of robot, storing it internally as a param"<<std::endl;

  urdf::Model robot_model; 
  if (!robot_model.initString( msg->urdf_xml_string))
  {std::cerr << "ERROR: Could not generate robot model" << std::endl;}

  typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joints_mapType; 
  for( joints_mapType::const_iterator it = robot_model.joints_.begin(); it!=robot_model.joints_.end(); it++)
  { 
	  if(it->second->type!=6) // All joints that not of the type FIXED.
           robot->joint_names_.push_back(it->first);//Joint names are sorted in alphabetical order within the urdf::Model structure.
  }
 }//end onMessage

}//end namespace


void setJointVal(string jointname,double val,std::vector<std::string> &names,std::vector<float> &values)
{
 std::vector<std::string>::const_iterator found;
 found = std::find(names.begin(), names.end(), jointname);
 if (found != names.end()) {
     int index = found - names.begin();
     values[index] = (float)val;
  }
}

int main(int argc, char ** argv)
{
  // Creates a subcription to ROBOT_MODEL channel to download the robot_model.
  // The received robot_model is instantiated into a RobotModel object. This object
  // contains the urdf_string as well as a list of joint names that are available on
  // the robot.
  test_est_robot_state_publisher::RobotModel* robot = new test_est_robot_state_publisher::RobotModel;

  lcm::Subscription* robot_model_subcription_;
  robot_model_subcription_ = robot->lcm.subscribeFunction("ROBOT_MODEL", test_est_robot_state_publisher::onMessage, robot);

  while(robot->lcm.handle()==-1);// wait for one message, wait until you get a success.
  robot->lcm.unsubscribe(robot_model_subcription_); // Stop listening to ROBOT_MODEL.

   
  std::cout<< "Received URDF of robot"<<std::endl;
  std::cout<< "Number of Joints: " << robot->joint_names_.size() <<std::endl;

 
// Start reading joint angles via gazebo/robot API for the joints mentioned in the URDF
// description and start publishing them 
  lcm::LCM lcm;
    if(!lcm.good())
        return 1;


  drc::robot_state_t message;

// Get joint angles using  gazebo/robot API for all joints in robot->joint_names_
  while(true)
  {

    struct timeval tv;
    gettimeofday (&tv, NULL);
    message.utime = (int64_t) tv.tv_sec * 1000000 + tv.tv_usec; 
    
    message.pose.translation.x = 0;
    message.pose.translation.y = 0;
    message.pose.translation.z = 0.8875;
    message.pose.rotation.x = 0;
    message.pose.rotation.y = 0;
    message.pose.rotation.z = 0;
    message.pose.rotation.w = 1;
    
    message.twist.linear_velocity.x =0;
    message.twist.linear_velocity.y =0;
    message.twist.linear_velocity.z =0;
    message.twist.angular_velocity.x =0;
    message.twist.angular_velocity.y =0;
    message.twist.angular_velocity.z =0;


    message.num_joints = robot->joint_names_.size();

    for(std::vector<std::string>::size_type i = 0; i != robot->joint_names_.size(); i++) 
    {
      //std::cout<< robot->joint_names_[i] <<std::endl;
      message.joint_name.push_back(robot->joint_names_[i]); // Joint names available in alphabetical order.
      message.joint_position.push_back(0.0);
      message.joint_velocity.push_back(0);
      message.joint_effort.push_back(0);
    }
    //atlas bdi fp
    setJointVal("l_arm_usy",0.2433,message.joint_name,message.joint_position);
    setJointVal("l_arm_shx",-1.3518,message.joint_name,message.joint_position);
    setJointVal("l_arm_ely",1.999,message.joint_name,message.joint_position);
    setJointVal("l_arm_elx",1.0032,message.joint_name,message.joint_position);
    setJointVal("l_arm_uwy",0.0000,message.joint_name,message.joint_position);
    setJointVal("l_leg_hpz",0.0004,message.joint_name,message.joint_position);
    setJointVal("l_leg_hpx",0.0501,message.joint_name,message.joint_position);
    setJointVal("l_leg_hpy",-0.3754,message.joint_name,message.joint_position);
    setJointVal("l_leg_kny",0.7603,message.joint_name,message.joint_position);
    setJointVal("l_leg_aky",-0.3844,message.joint_name,message.joint_position);
    setJointVal("l_leg_akx",-0.0501,message.joint_name,message.joint_position);
    setJointVal("l_arm_mwx",0.0006,message.joint_name,message.joint_position);

    setJointVal("r_arm_usy",0.2433,message.joint_name,message.joint_position);
    setJointVal("r_arm_shx",1.3518,message.joint_name,message.joint_position);
    setJointVal("r_arm_ely",1.999,message.joint_name,message.joint_position);
    setJointVal("r_arm_elx",-1.0032,message.joint_name,message.joint_position);
    setJointVal("r_arm_uwy",0.0000,message.joint_name,message.joint_position);
    setJointVal("r_leg_hpz",0.0002,message.joint_name,message.joint_position);
    setJointVal("r_leg_hpx",-0.0502,message.joint_name,message.joint_position);
    setJointVal("r_leg_hpy",-0.3754,message.joint_name,message.joint_position);
    setJointVal("r_leg_kny",0.7602,message.joint_name,message.joint_position);
    setJointVal("r_leg_aky",-0.3843,message.joint_name,message.joint_position);
    setJointVal("r_leg_akx",0.0502,message.joint_name,message.joint_position);
    setJointVal("r_arm_mwx",-0.0006,message.joint_name,message.joint_position);
    setJointVal("left_f0_j0",0.000,message.joint_name,message.joint_position);
    setJointVal("left_f0_j1",0.001,message.joint_name,message.joint_position);
    setJointVal("left_f0_j2",0.002,message.joint_name,message.joint_position);
    setJointVal("left_f1_j0",0.000,message.joint_name,message.joint_position);
    setJointVal("left_f1_j1",0.001,message.joint_name,message.joint_position);
    setJointVal("left_f1_j2",0.002,message.joint_name,message.joint_position);
    setJointVal("left_f2_j0",0.000,message.joint_name,message.joint_position);
    setJointVal("left_f2_j1",0.001,message.joint_name,message.joint_position);
    setJointVal("left_f2_j2",0.002,message.joint_name,message.joint_position);
    setJointVal("left_f3_j0",0.000,message.joint_name,message.joint_position);
    setJointVal("left_f3_j1",0.001,message.joint_name,message.joint_position);
    setJointVal("left_f3_j2",0.002,message.joint_name,message.joint_position);
    setJointVal("right_f0_j0",0.003,message.joint_name,message.joint_position);
    setJointVal("right_f0_j1",0.004,message.joint_name,message.joint_position);
    setJointVal("right_f0_j2",0.005,message.joint_name,message.joint_position);
    setJointVal("right_f1_j0",0.003,message.joint_name,message.joint_position);
    setJointVal("right_f1_j1",0.004,message.joint_name,message.joint_position);
    setJointVal("right_f1_j2",0.005,message.joint_name,message.joint_position);
    setJointVal("right_f2_j0",0.003,message.joint_name,message.joint_position);
    setJointVal("right_f2_j1",0.004,message.joint_name,message.joint_position);
    setJointVal("right_f2_j2",0.005,message.joint_name,message.joint_position);
    setJointVal("right_f3_j0",0.003,message.joint_name,message.joint_position);
    setJointVal("right_f3_j1",0.004,message.joint_name,message.joint_position);
    setJointVal("right_f3_j2",0.005,message.joint_name,message.joint_position);

    
    // Publish
    lcm.publish("EST_ROBOT_STATE", &message);
    
    // clear vectors
    message.joint_name.clear();
    message.joint_position.clear();
    message.joint_velocity.clear();
    message.joint_effort.clear();

    usleep(10000); // publish at 100 hz.
   } 

   delete robot; 
   return 0;
 
}


