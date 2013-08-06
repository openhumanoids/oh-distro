// depricated, june 2013

// Find the Head to Hokuyo Frame to complete the transform tree for bot-frames
// Currently delta XYZ is hard coded
// The output of this can be verified versus joint2frames
#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <map>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm-cpp.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/multisense.hpp"

using namespace std;
using namespace boost;
using namespace Eigen;

class joints2frames{
  public:
    joints2frames(boost::shared_ptr<lcm::LCM> &publish_lcm);
    
    ~joints2frames(){
    }
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    //void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::robot_state_prefeb2013_t* msg);
    
    void joint_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::joint_t* msg);
    
    void sendJoint(double joint_position, int64_t utime);
};    

/////////////////////////////////////
/*
joints2frames::joints2frames(boost::shared_ptr<lcm::LCM> &publish_lcm):
          lcm_(publish_lcm){
  lcm_->subscribe("EST_ROBOT_STATE",&joints2frames::robot_state_handler,this);  
} */

joints2frames::joints2frames(boost::shared_ptr<lcm::LCM> &publish_lcm):
          lcm_(publish_lcm){
  // Original Sensor:
  lcm_->subscribe("OLD_TRUE_ROBOT_STATE",&joints2frames::robot_state_handler,this);  
  
  lcm_->subscribe("MULTISENSE_JOINT",&joints2frames::joint_handler,this);  
}

Eigen::Isometry3d getPose(double lidar_angle){
  // Convert Euler to Isometry3d:
  Matrix3d m;
  m = AngleAxisd (0, Vector3d::UnitZ ())
                  * AngleAxisd (0, Vector3d::UnitY ())
                  * AngleAxisd (lidar_angle, Vector3d::UnitX ());  
  Eigen::Isometry3d pose =  Eigen::Isometry3d::Identity();
  pose *= m;  
  pose.translation()  << -0.0446, 0, 0.088; /// NB hard coded
  return pose;
}


void joints2frames::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                                        const  multisense::robot_state_prefeb2013_t* msg){
  
  sendJoint( msg->joint_position[0], msg->utime);
}

void joints2frames::joint_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
                                        const  multisense::joint_t* msg){
  sendJoint( msg->position - 1.10, msg->utime);
}


void joints2frames::sendJoint(double joint_position, int64_t utime){
  Eigen::Isometry3d head_to_hokuyo_link = getPose(joint_position);

  
  bot_core::rigid_transform_t tf;
  tf.utime = utime;
  tf.trans[0] = head_to_hokuyo_link.translation().x();
  tf.trans[1] = head_to_hokuyo_link.translation().y();
  tf.trans[2] = head_to_hokuyo_link.translation().z();
  Eigen::Quaterniond quat = Eigen::Quaterniond( head_to_hokuyo_link.rotation() );
  tf.quat[0] = quat.w();
  tf.quat[1] = quat.x();
  tf.quat[2] = quat.y();
  tf.quat[3] = quat.z();
  lcm_->publish("HEAD_TO_HOKUYO_LINK", &tf);     
    
  cout << head_to_hokuyo_link.translation().x() << " "
        << head_to_hokuyo_link.translation().y() << " "
        << head_to_hokuyo_link.translation().z() << " | "
        << quat.w() << " "
        << quat.x() << " "
        << quat.y() << " "
        << quat.z() << " Head to Hokuyo\n";    
}

int main(int argc, char ** argv){
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  joints2frames app(lcm);
  while(0 == lcm->handle());
  return 0;
}
