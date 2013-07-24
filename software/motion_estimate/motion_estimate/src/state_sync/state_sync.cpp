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

  lcm_->subscribe("EST_ROBOT_STATE",&state_sync::robotStateHandler,this);  
  lcm_->subscribe("STATE_MULTISENSE",&state_sync::multisenseHandler,this);  
  
  
  head_names_ = {"hokuyo_joint", 
    "pre_spindle_cal_x_joint", "pre_spindle_cal_y_joint", "pre_spindle_cal_z_joint",
    "pre_spindle_cal_roll_joint", "pre_spindle_cal_pitch_joint", "pre_spindle_cal_yaw_joint",
    "post_spindle_cal_x_joint", "post_spindle_cal_y_joint", "post_spindle_cal_z_joint",
    "post_spindle_cal_roll_joint", "post_spindle_cal_pitch_joint", "post_spindle_cal_yaw_joint"};
  
  /*
  head_joints_.position.push_back(0);
  head_joints_.velocity.push_back(0);
  head_joints_.effort.push_back(0);
  head_joints_.name.push_back("hokuyo_joint");*/
}

void state_sync::robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
   
}

void state_sync::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg){
  std::cout << "got multisense\n";
  head_joints_.position = msg->position;
  head_joints_.velocity = msg->velocity;
  head_joints_.effort = msg->effort;
  head_joints_.name = head_names_;

  publishRobotState(msg->utime);
}


void state_sync::appendJoints(drc::robot_state_t& msg_out, Joints joints){
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

void state_sync::publishRobotState(int64_t utime_in){
  
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
  //appendJoints(robot_state_msg, robot_joint_states_,false);
  appendJoints(robot_state_msg, head_joints_);
  //appendJoints(robot_state_msg, l_hand_joint_states_,false);
  //appendJoints(robot_state_msg, r_hand_joint_states_,false);
  std::cout << robot_state_msg.joint_name.size() << " nj\n";
  robot_state_msg.num_joints = robot_state_msg.joint_name.size();
  
  // Limb Sensor states
  //appendLimbSensor(robot_state_msg, end_effector_sensors_);
  //robot_state_msg.contacts.num_contacts = robot_state_msg.contacts.contact_torque.size();
  robot_state_msg.contacts.num_contacts =0;
    
  lcm_->publish("EST_ROBOT_STATE", &robot_state_msg);    

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

