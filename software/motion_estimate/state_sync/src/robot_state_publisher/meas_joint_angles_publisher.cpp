// file: joint_angle_publisher.cpp
//
// Communicates to robot API to get joint angles and then 
// publishes them on MEAS_JOINT_ANGLES lcm channel.
// The publisher requires access to the urdf robot model. 
// At startup, the process creates a subcription to ROBOT_MODEL lcm channel,
// waits until it receives the urdf string and then unsubcribes from this
// channel.
//


#include <iostream>
#include <stdint.h> 
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include <urdf/model.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/joint_angles_t.hpp"
#include "lcmtypes/drc/robot_urdf_t.hpp"

namespace meas_joint_angles_publisher {
  
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
  meas_joint_angles_publisher::RobotModel* robot = new meas_joint_angles_publisher::RobotModel;

  lcm::Subscription* robot_model_subcription_;
  robot_model_subcription_ = robot->lcm.subscribeFunction("ROBOT_MODEL", meas_joint_angles_publisher::onMessage, robot);

  while(robot->lcm.handle()==-1);// wait for one message, wait until you get a success.
  robot->lcm.unsubscribe(robot_model_subcription_); // Stop listening to ROBOT_MODEL.

   
  std::cout<< "Received URDF of robot [" <<robot->robot_name <<"] "<<std::endl;
  std::cout<< "Number of Joints: " << robot->joint_names_.size() <<std::endl;

 
// Start reading joint angles via gazebo/robot API for the joints mentioned in the URDF
// description and start publishing them 
  lcm::LCM lcm;
    if(!lcm.good())
        return 1;


  drc::joint_angles_t message;

// Only define robot_name and joint_names once.
  
  
// Get joint angles using  gazebo/robot API for all joints in robot->joint_names_
  while(true)
  {
    //message.timestamp = time(NULL);
    struct timeval tv;
    gettimeofday (&tv, NULL);
    message.utime = (int64_t) tv.tv_sec * 1000000 + tv.tv_usec; // TODO: replace with bot_timestamp_now() from bot_core
    message.robot_name = robot -> robot_name;
    message.num_joints = robot->joint_names_.size();
   
    for(std::vector<std::string>::size_type i = 0; i != robot->joint_names_.size(); i++) 
    {
      //std::cout<< robot->joint_names_[i] <<std::endl;
      message.joint_name.push_back(robot->joint_names_[i]); // Joint names available in alphabetical order.
      //TODO: INSERT ROBOT/GAZEBO API HERE?
	double val = 0;

      // for debugging ..
     /*if(robot->joint_names_[i] == "LShoulderRoll")
           val = -30*3.14/180;
      if(robot->joint_names_[i] == "RShoulderRoll")
           val = 30*3.14/180;
      if(robot->joint_names_[i] == "LElbowPitch")
           val = -30*3.14/180;
    if(robot->joint_names_[i] == "RElbowPitch")
           val = -30*3.14/180;*/
      message.joint_position.push_back(val);

    }


    // Publishing joint angles.
    lcm.publish("MEAS_JOINT_ANGLES", &message);
    message.joint_name.clear();
    message.joint_position.clear();
    //TODO: better Timing.
    usleep(10000); // publish at 100 hz.
   } 

   delete robot; 
   return 0;
 
}


