#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>

#include "state_sync.hpp"
#include <ConciseArgs>

using namespace std;
#define DO_TIMING_PROFILE FALSE

/////////////////////////////////////

state_sync::state_sync(boost::shared_ptr<lcm::LCM> &lcm_):lcm_(lcm_){
  lcm_->subscribe("MULTISENSE_STATE",&state_sync::multisenseHandler,this);  
  lcm_->subscribe("SANDIA_LEFT_STATE",&state_sync::sandiaLeftHandler,this);  
  lcm_->subscribe("SANDIA_RIGHT_STATE",&state_sync::sandiaRightHandler,this);  
  lcm_->subscribe("ATLAS_STATE",&state_sync::atlasHandler,this);  
}


void state_sync::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg){
  //std::cout << "got multisense\n";
  head_joints_.name = msg->joint_name;
  head_joints_.position = msg->joint_position;
  head_joints_.velocity = msg->joint_velocity;
  head_joints_.effort = msg->joint_effort;
}

void state_sync::sandiaLeftHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::sandia_state_t* msg){
  //std::cout << "got sandiaLeftHandler\n";
  sandia_left_joints_.name = msg->joint_name;
  sandia_left_joints_.position = msg->joint_position;
  sandia_left_joints_.velocity = msg->joint_velocity;
  sandia_left_joints_.effort = msg->joint_effort;
}

void state_sync::sandiaRightHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::sandia_state_t* msg){
  //std::cout << "got sandiaRightHandler\n";
  sandia_right_joints_.name = msg->joint_name;
  sandia_right_joints_.position = msg->joint_position;
  sandia_right_joints_.velocity = msg->joint_velocity;
  sandia_right_joints_.effort = msg->joint_effort;
}

void state_sync::atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg){
  //std::cout << "got atlasHandler\n";
  atlas_joints_.name = msg->joint_name;
  atlas_joints_.position = msg->joint_position;
  atlas_joints_.velocity = msg->joint_velocity;
  atlas_joints_.effort = msg->joint_effort;
  
  publishRobotState(msg->utime, msg->force_torque);
  publishRobotState_VRC(msg->utime, msg->force_torque);
  
}


void state_sync::publishRobotState(int64_t utime_in,  const  drc::force_torque_t& force_torque_msg){
  
  drc::state_t robot_state_msg;
  robot_state_msg.utime = utime_in;
  
  // Pelvis Pose:
  robot_state_msg.origin_position.translation.x =0;
  robot_state_msg.origin_position.translation.y =0;
  robot_state_msg.origin_position.translation.z =0;
  robot_state_msg.origin_position.rotation.w = 1;
  robot_state_msg.origin_position.rotation.x = 0;
  robot_state_msg.origin_position.rotation.y = 0;
  robot_state_msg.origin_position.rotation.z = 0;

  robot_state_msg.origin_twist.linear_velocity.x  = 0;
  robot_state_msg.origin_twist.linear_velocity.y  = 0;
  robot_state_msg.origin_twist.linear_velocity.z  = 0;
  robot_state_msg.origin_twist.angular_velocity.x = 0;
  robot_state_msg.origin_twist.angular_velocity.y = 0;
  robot_state_msg.origin_twist.angular_velocity.z = 0;

  // Joint States:
  appendJoints(robot_state_msg, atlas_joints_);
  appendJoints(robot_state_msg, head_joints_);
  appendJoints(robot_state_msg, sandia_left_joints_);
  appendJoints(robot_state_msg, sandia_right_joints_);
  //std::cout << robot_state_msg.joint_name.size() << " Number of Joints\n";
  robot_state_msg.num_joints = robot_state_msg.joint_name.size();
  
  // Limb Sensor states
  robot_state_msg.force_torque = force_torque_msg;
  
  lcm_->publish("TRUE_ROBOT_STATE", &robot_state_msg);    
}

void state_sync::appendJoints(drc::state_t& msg_out, Joints joints){
  for (size_t i = 0; i < joints.position.size(); i++)  {
    msg_out.joint_name.push_back( joints.name[i] );
    msg_out.joint_position.push_back( joints.position[i] );
    msg_out.joint_velocity.push_back( joints.velocity[i]);
    msg_out.joint_effort.push_back( joints.effort[i] );
  }
}


/////////////////////////////////////////////////////////////////////////
//// VRC-era publishes ///////////////////////////////////////////////////
void state_sync::publishRobotState_VRC(int64_t utime_in,  const  drc::force_torque_t& force_torque_msg){
  
  drc::robot_state_t robot_state_msg;
  robot_state_msg.utime = utime_in;
  robot_state_msg.robot_name = "atlas";
  
  // Pelvis Pose:
  robot_state_msg.origin_position.translation.x =0;
  robot_state_msg.origin_position.translation.y =0;
  robot_state_msg.origin_position.translation.z =0;
  robot_state_msg.origin_position.rotation.w = 1;
  robot_state_msg.origin_position.rotation.x = 0;
  robot_state_msg.origin_position.rotation.y = 0;
  robot_state_msg.origin_position.rotation.z = 0;

  robot_state_msg.origin_twist.linear_velocity.x  = 0;
  robot_state_msg.origin_twist.linear_velocity.y  = 0;
  robot_state_msg.origin_twist.linear_velocity.z  = 0;
  robot_state_msg.origin_twist.angular_velocity.x = 0;
  robot_state_msg.origin_twist.angular_velocity.y = 0;
  robot_state_msg.origin_twist.angular_velocity.z = 0;
  for(int i = 0; i < 6; i++)  {
    for(int j = 0; j < 6; j++) {
      robot_state_msg.origin_cov.position_cov[i][j] = 0;
      robot_state_msg.origin_cov.twist_cov[i][j] = 0;
    }
  }

  // Joint States:
  appendJoints_VRC(robot_state_msg, atlas_joints_);
  appendJoints_VRC(robot_state_msg, head_joints_);
  appendJoints_VRC(robot_state_msg, sandia_left_joints_);
  appendJoints_VRC(robot_state_msg, sandia_right_joints_);
  //std::cout << robot_state_msg.joint_name.size() << " Number of Joints\n";
  robot_state_msg.num_joints = robot_state_msg.joint_name.size();
  
  // Limb Sensor states
  robot_state_msg.contacts = setContacts_VRC(force_torque_msg);
  
  lcm_->publish("TRUE_ROBOT_STATE_OLD", &robot_state_msg);    
}


drc::contact_state_t state_sync::setContacts_VRC(const  drc::force_torque_t& msg_in){
  drc::contact_state_t contacts;
  {
    drc::vector_3d_t l_foot_force;
    l_foot_force.x = 0;                       l_foot_force.y = 0;                       l_foot_force.z = msg_in.l_foot_force_z;
    drc::vector_3d_t l_foot_torque;
    l_foot_torque.x = msg_in.l_foot_torque_x; l_foot_torque.y = msg_in.l_foot_torque_y; l_foot_torque.z = 0;
    contacts.id.push_back("l_foot");
    contacts.contact_force.push_back(l_foot_force);
    contacts.contact_torque.push_back(l_foot_torque);  
  }

  
  {
    drc::vector_3d_t r_foot_force;
    r_foot_force.x = 0;                       r_foot_force.y = 0;                       r_foot_force.z = msg_in.r_foot_force_z;
    drc::vector_3d_t r_foot_torque;
    r_foot_torque.x = msg_in.r_foot_torque_x; r_foot_torque.y = msg_in.r_foot_torque_y; r_foot_torque.z = 0;
    contacts.id.push_back("r_foot");
    contacts.contact_force.push_back(r_foot_force);
    contacts.contact_torque.push_back(r_foot_torque);  
  }  

  {
    drc::vector_3d_t l_hand_force;
    l_hand_force.x = msg_in.l_hand_force[0]; l_hand_force.y = msg_in.l_hand_force[1]; l_hand_force.z = msg_in.l_hand_force[2];
    drc::vector_3d_t l_hand_torque;
    l_hand_torque.x = msg_in.l_hand_torque[0]; l_hand_torque.y = msg_in.l_hand_torque[1]; l_hand_torque.z = msg_in.l_hand_torque[2];
    contacts.id.push_back("l_hand");
    contacts.contact_force.push_back(l_hand_force);
    contacts.contact_torque.push_back(l_hand_torque);  
  }
    
  {
    drc::vector_3d_t r_hand_force;
    r_hand_force.x = msg_in.r_hand_force[0]; r_hand_force.y = msg_in.r_hand_force[1]; r_hand_force.z = msg_in.r_hand_force[2];
    drc::vector_3d_t r_hand_torque;
    r_hand_torque.x = msg_in.r_hand_torque[0]; r_hand_torque.y = msg_in.r_hand_torque[1]; r_hand_torque.z = msg_in.r_hand_torque[2];
    contacts.id.push_back("r_hand");
    contacts.contact_force.push_back(r_hand_force);
    contacts.contact_torque.push_back(r_hand_torque);  
  } 
  
  contacts.num_contacts = contacts.id.size();
  
  return contacts;
}

void state_sync::appendJoints_VRC(drc::robot_state_t& msg_out, Joints joints){
  drc::joint_covariance_t j_cov;
  j_cov.variance = 0;

  for (size_t i = 0; i < joints.position.size(); i++)  {
    msg_out.joint_name.push_back( joints.name[i] );
    msg_out.joint_position.push_back( joints.position[i] );
    msg_out.joint_velocity.push_back( joints.velocity[i]);
    msg_out.measured_effort.push_back( joints.effort[i] );
    msg_out.joint_cov.push_back(j_cov);
  }
}



int
main(int argc, char ** argv){
  bool labels = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(labels, "l", "labels","Frame Labels - show no not");
  opt.parse();
  
  std::cout << "labels: " << labels << "\n";

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  state_sync app(lcm);
  while(0 == lcm->handle());
  return 0;
}

