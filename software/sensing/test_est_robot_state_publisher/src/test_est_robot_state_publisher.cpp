// Test program that listens to robot urdf and publishes its state on EST_ROBOT_STATE with zero joint angles

#include <iostream>
#include <stdint.h> 
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include <urdf/model.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

namespace test_est_robot_state_publisher {
  
class RobotModel {
 public:
   lcm::LCM lcm;
   std::string robot_name;
   std::string urdf_xml_string; 
   std::vector<std::string> joint_names_;
 };

void onMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg, RobotModel* robot) {
  // Received robot urdf string. Store it internally and get all available joints.

   robot->robot_name      = msg->robot_name;
   robot->urdf_xml_string = msg->urdf_xml_string;
  std::cout<<"Received urdf_xml_string of robot ["<<msg->robot_name <<"], storing it internally as a param"<<std::endl;

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

   
  std::cout<< "Received URDF of robot [" <<robot->robot_name <<"] "<<std::endl;
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
    message.robot_name = robot -> robot_name;
    
    message.origin_position.translation.x = 0;
    message.origin_position.translation.y = 0;
    message.origin_position.translation.z = 0;
    message.origin_position.rotation.x = 0;
    message.origin_position.rotation.y = 0;
    message.origin_position.rotation.z = 0;
    message.origin_position.rotation.w = 1;
    
    message.origin_twist.linear_velocity.x =0;
    message.origin_twist.linear_velocity.y =0;
    message.origin_twist.linear_velocity.z =0;
    message.origin_twist.angular_velocity.x =0;
    message.origin_twist.angular_velocity.y =0;
    message.origin_twist.angular_velocity.z =0;

    int i,j;
    for(i = 0; i < 6; i++)  {
       for(j = 0; j < 6; j++) {
             message.origin_cov.position_cov[i][j] = 0;
	     message.origin_cov.twist_cov[i][j] = 0;
    }
    }

    drc::joint_covariance_t j_cov;
    j_cov.variance = 0;

    message.num_joints = robot->joint_names_.size();

    for(std::vector<std::string>::size_type i = 0; i != robot->joint_names_.size(); i++) 
    {
      //std::cout<< robot->joint_names_[i] <<std::endl;
      message.joint_name.push_back(robot->joint_names_[i]); // Joint names available in alphabetical order.
      message.joint_position.push_back(0.0);
      message.joint_velocity.push_back(0);
      message.measured_effort.push_back(0);
      message.joint_cov.push_back(j_cov);
    }


    // dummy ground contact states
    message.contacts.num_contacts =0;
    message.contacts.id.push_back("dummy");
    for (int i=0; i< message.contacts.num_contacts; i++){
        message.contacts.inContact.push_back(0);
        drc::vector_3d_t f_zero;
        f_zero.x = 0;f_zero.y = 0;f_zero.z = 0;
        message.contacts.contactForce.push_back(f_zero);
    }
    // Publish
    lcm.publish("EST_ROBOT_STATE", &message);
    
    // clear vectors
    message.joint_name.clear();
    message.joint_position.clear();
    message.joint_velocity.clear();
    message.measured_effort.clear();
    message.joint_cov.clear();
    message.contacts.id.clear();
    message.contacts.inContact.clear();
    message.contacts.contactForce.clear();
    
    usleep(10000); // publish at 100 hz.
   } 

   delete robot; 
   return 0;
 
}


