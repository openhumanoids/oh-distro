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
  
  
  head_names_ = {"hokuyo_joint", 
    "pre_spindle_cal_x_joint", "pre_spindle_cal_y_joint", "pre_spindle_cal_z_joint",
    "pre_spindle_cal_roll_joint", "pre_spindle_cal_pitch_joint", "pre_spindle_cal_yaw_joint",
    "post_spindle_cal_x_joint", "post_spindle_cal_y_joint", "post_spindle_cal_z_joint",
    "post_spindle_cal_roll_joint", "post_spindle_cal_pitch_joint", "post_spindle_cal_yaw_joint"};

  // Old Strings:
  atlas_names_ = {"back_lbz", "back_mby", "back_ubx", "neck_ay",
     "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny",
     "l_leg_uay", "l_leg_lax", "r_leg_uhz", "r_leg_mhx",
     "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax",
     "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx",
     "l_arm_uwy", "l_arm_mwx", "r_arm_usy", "r_arm_shx",
     "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"};
    
  // New Strings
/*  atlas_names_ = {"back_bkz", "back_bky", "back_bkx", "neck_ay",
    "l_leg_hpz", "l_leg_hpx", "l_leg_hpy", "l_leg_kny", 
    "l_leg_aky", "l_leg_akx", "r_leg_hpz", "r_leg_hpx",
    "r_leg_hpy", "r_leg_kny", "r_leg_aky", "r_leg_akx",
    "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx",
    "l_arm_uwy", "l_arm_mwx", "r_arm_usy", "r_arm_shx",
    "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"};    
    */

  sandia_left_names_ = {"left_f0_j0", "left_f0_j1", "left_f0_j2", "left_f1_j0",
    "left_f1_j1", "left_f1_j2", "left_f2_j0", "left_f2_j1",
    "left_f2_j2", "left_f3_j0", "left_f3_j1", "left_f3_j2"};
    
  sandia_right_names_ = {"right_f0_j0", "right_f0_j1", "right_f0_j2", "right_f1_j0",
    "right_f1_j1", "right_f1_j2", "right_f2_j0", "right_f2_j1",
    "right_f2_j2", "right_f3_j0", "right_f3_j1", "right_f3_j2"};
    
  /*
  head_joints_.position.push_back(0);
  head_joints_.velocity.push_back(0);
  head_joints_.effort.push_back(0);
  head_joints_.name.push_back("hokuyo_joint");*/
}


void state_sync::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg){
  //std::cout << "got multisense\n";
  head_joints_.type = msg->joint_type;
  head_joints_.position = msg->joint_position;
  head_joints_.velocity = msg->joint_velocity;
  head_joints_.effort = msg->joint_effort;
  head_joints_.name = head_names_;
}

void state_sync::sandiaLeftHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::sandia_state_t* msg){
  //std::cout << "got sandiaLeftHandler\n";
  sandia_left_joints_.type = msg->joint_type;
  sandia_left_joints_.position = msg->joint_position;
  sandia_left_joints_.velocity = msg->joint_velocity;
  sandia_left_joints_.effort = msg->joint_effort;
  sandia_left_joints_.name = sandia_left_names_;
}

void state_sync::sandiaRightHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::sandia_state_t* msg){
  //std::cout << "got sandiaRightHandler\n";
  sandia_right_joints_.type = msg->joint_type;
  sandia_right_joints_.position = msg->joint_position;
  sandia_right_joints_.velocity = msg->joint_velocity;
  sandia_right_joints_.effort = msg->joint_effort;
  sandia_right_joints_.name = sandia_right_names_;
}

void state_sync::atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg){
  //std::cout << "got atlasHandler\n";
  atlas_joints_.type = msg->joint_type;
  atlas_joints_.position = msg->joint_position;
  atlas_joints_.velocity = msg->joint_velocity;
  atlas_joints_.effort = msg->joint_effort;
  atlas_joints_.name = atlas_names_;
  
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
  //std::cout << robot_state_msg.joint_type.size() << " Number of Joints\n";
  robot_state_msg.num_joints = robot_state_msg.joint_type.size();
  
  // Limb Sensor states
  robot_state_msg.force_torque = force_torque_msg;
  
  lcm_->publish("ROBOT_STATE_TRUE", &robot_state_msg);    
}

void state_sync::appendJoints(drc::state_t& msg_out, Joints joints){
  for (size_t i = 0; i < joints.position.size(); i++)  {
    msg_out.joint_type.push_back( joints.type[i] );
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
  
  lcm_->publish("TRUE_ROBOT_STATE", &robot_state_msg);    
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

