#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>

#include "state_sync.hpp"
#include <ConciseArgs>

using namespace std;
#define DO_TIMING_PROFILE FALSE

/////////////////////////////////////

state_sync::state_sync(boost::shared_ptr<lcm::LCM> &lcm_, bool standalone_head_, bool bdi_motion_estimate_):
   lcm_(lcm_), standalone_head_(standalone_head_),
   is_sandia_left_(false),is_sandia_right_(false),	
   bdi_motion_estimate_(bdi_motion_estimate_){
  lcm_->subscribe("MULTISENSE_STATE",&state_sync::multisenseHandler,this);  
  lcm_->subscribe("SANDIA_LEFT_STATE",&state_sync::sandiaLeftHandler,this);  
  lcm_->subscribe("SANDIA_RIGHT_STATE",&state_sync::sandiaRightHandler,this);  
  lcm_->subscribe("IROBOT_LEFT_STATE",&state_sync::irobotLeftHandler,this);  
  lcm_->subscribe("IROBOT_RIGHT_STATE",&state_sync::irobotRightHandler,this);  
  lcm_->subscribe("ATLAS_STATE",&state_sync::atlasHandler,this);  
  
  lcm_->subscribe("POSE_BDI",&state_sync::poseBDIHandler,this); 
  pose_BDI_.utime =0; // use this to signify un-initalised
}


void state_sync::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg){
  //std::cout << "got multisense\n";
  head_joints_.name = msg->joint_name;
  head_joints_.position = msg->joint_position;
  head_joints_.velocity = msg->joint_velocity;
  head_joints_.effort = msg->joint_effort;
  
  if (standalone_head_){
    drc::force_torque_t force_torque_msg;
    publishRobotState(msg->utime, force_torque_msg);
  }
  
}

void state_sync::sandiaLeftHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg){
  is_sandia_left_ = true;
  //std::cout << "got sandiaLeftHandler\n";
  sandia_left_joints_.name = msg->joint_name;
  sandia_left_joints_.position = msg->joint_position;
  sandia_left_joints_.velocity = msg->joint_velocity;
  sandia_left_joints_.effort = msg->joint_effort;
}

void state_sync::sandiaRightHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg){
  is_sandia_right_ = true;
  //std::cout << "got sandiaRightHandler\n";
  sandia_right_joints_.name = msg->joint_name;
  sandia_right_joints_.position = msg->joint_position;
  sandia_right_joints_.velocity = msg->joint_velocity;
  sandia_right_joints_.effort = msg->joint_effort;
}

void state_sync::irobotLeftHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg){
  //std::cout << "got sandiaLeftHandler\n";
  irobot_left_joints_.name = msg->joint_name;
  irobot_left_joints_.position = msg->joint_position;
  irobot_left_joints_.velocity = msg->joint_velocity;
  irobot_left_joints_.effort = msg->joint_effort;
}

void state_sync::irobotRightHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg){
  //std::cout << "got sandiaRightHandler\n";
  irobot_right_joints_.name = msg->joint_name;
  irobot_right_joints_.position = msg->joint_position;
  irobot_right_joints_.velocity = msg->joint_velocity;
  irobot_right_joints_.effort = msg->joint_effort;
}

void state_sync::atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg){
  //std::cout << "got atlasHandler\n";
  atlas_joints_.name = msg->joint_name;
  atlas_joints_.position = msg->joint_position;
  atlas_joints_.velocity = msg->joint_velocity;
  atlas_joints_.effort = msg->joint_effort;
  
  publishRobotState(msg->utime, msg->force_torque);
}


void state_sync::poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  pose_BDI_.utime = msg->utime;
  pose_BDI_.pos = Eigen::Vector3d( msg->pos[0],  msg->pos[1],  msg->pos[2] );
  pose_BDI_.vel = Eigen::Vector3d( msg->vel[0],  msg->vel[1],  msg->vel[2] );
  pose_BDI_.orientation = Eigen::Vector4d( msg->orientation[0],  msg->orientation[1],  msg->orientation[2],  msg->orientation[3] );
  pose_BDI_.rotation_rate = Eigen::Vector3d( msg->rotation_rate[0],  msg->rotation_rate[1],  msg->rotation_rate[2] );
  pose_BDI_.accel = Eigen::Vector3d( msg->accel[0],  msg->accel[1],  msg->accel[2] );  
}



bool state_sync::insertPoseBDI(drc::robot_state_t& msg){
  // TODO: add comparison of msg->utime and pose_BDI_'s utime  
  
  msg.pose.translation.x = pose_BDI_.pos[0];
  msg.pose.translation.y = pose_BDI_.pos[1];
  msg.pose.translation.z = pose_BDI_.pos[2];
  msg.pose.rotation.w = pose_BDI_.orientation[0];
  msg.pose.rotation.x = pose_BDI_.orientation[1];
  msg.pose.rotation.y = pose_BDI_.orientation[2];
  msg.pose.rotation.z = pose_BDI_.orientation[3];

  msg.twist.linear_velocity.x = pose_BDI_.vel[0];
  msg.twist.linear_velocity.y = pose_BDI_.vel[1];
  msg.twist.linear_velocity.z = pose_BDI_.vel[2];
  
  msg.twist.angular_velocity.x = pose_BDI_.rotation_rate[0];
  msg.twist.angular_velocity.y = pose_BDI_.rotation_rate[1];
  msg.twist.angular_velocity.z = pose_BDI_.rotation_rate[2];
  
  return true;  
}
  


void state_sync::publishRobotState(int64_t utime_in,  const  drc::force_torque_t& force_torque_msg){
  
  drc::robot_state_t robot_state_msg;
  robot_state_msg.utime = utime_in;
  
  // Pelvis Pose:
  robot_state_msg.pose.translation.x =0;
  robot_state_msg.pose.translation.y =0;
  robot_state_msg.pose.translation.z =0;
  robot_state_msg.pose.rotation.w = 1;
  robot_state_msg.pose.rotation.x = 0;
  robot_state_msg.pose.rotation.y = 0;
  robot_state_msg.pose.rotation.z = 0;

  robot_state_msg.twist.linear_velocity.x  = 0;
  robot_state_msg.twist.linear_velocity.y  = 0;
  robot_state_msg.twist.linear_velocity.z  = 0;
  robot_state_msg.twist.angular_velocity.x = 0;
  robot_state_msg.twist.angular_velocity.y = 0;
  robot_state_msg.twist.angular_velocity.z = 0;

  // Joint States:
  appendJoints(robot_state_msg, atlas_joints_);
  appendJoints(robot_state_msg, head_joints_);
  
  if(is_sandia_left_)
    appendJoints(robot_state_msg, sandia_left_joints_);
  else
    appendJoints(robot_state_msg, irobot_left_joints_);
  
  if(is_sandia_right_)
    appendJoints(robot_state_msg, sandia_right_joints_);
  else
    appendJoints(robot_state_msg, irobot_right_joints_);  

  //std::cout << robot_state_msg.joint_name.size() << " Number of Joints\n";
  robot_state_msg.num_joints = robot_state_msg.joint_name.size();
  
  // Limb Sensor states
  robot_state_msg.force_torque = force_torque_msg;
  
  lcm_->publish("TRUE_ROBOT_STATE", &robot_state_msg);    
  if (bdi_motion_estimate_){
    if ( insertPoseBDI(robot_state_msg) ){
      lcm_->publish("EST_ROBOT_STATE", &robot_state_msg);    
    }
  }
}

void state_sync::appendJoints(drc::robot_state_t& msg_out, Joints joints){
  for (size_t i = 0; i < joints.position.size(); i++)  {
    msg_out.joint_name.push_back( joints.name[i] );
    msg_out.joint_position.push_back( joints.position[i] );
    msg_out.joint_velocity.push_back( joints.velocity[i]);
    msg_out.joint_effort.push_back( joints.effort[i] );
  }
}



int
main(int argc, char ** argv){
  bool standalone_head = false;
  bool bdi_motion_estimate = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(standalone_head, "l", "standalone_head","Standalone Head");
  opt.add(bdi_motion_estimate, "b", "bdi","Use POSE_BDI to make EST_ROBOT_STATE");
  opt.parse();
  
  std::cout << "standalone_head: " << standalone_head << "\n";

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  state_sync app(lcm, standalone_head,bdi_motion_estimate);
  while(0 == lcm->handle());
  return 0;
}

