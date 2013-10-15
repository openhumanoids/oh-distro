#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>

#include "state_sync.hpp"
#include <ConciseArgs>

using namespace std;
#define DO_TIMING_PROFILE FALSE

/////////////////////////////////////

state_sync::state_sync(boost::shared_ptr<lcm::LCM> &lcm_, 
                       bool standalone_head_, bool standalone_hand_,  
                       bool bdi_motion_estimate_, bool simulation_mode_,
                       bool use_transmission_joint_sensors_):
   lcm_(lcm_), 
   standalone_head_(standalone_head_), standalone_hand_(standalone_hand_),
   bdi_motion_estimate_(bdi_motion_estimate_), simulation_mode_(simulation_mode_),
   use_transmission_joint_sensors_(use_transmission_joint_sensors_){
  lcm_->subscribe("MULTISENSE_STATE",&state_sync::multisenseHandler,this);  
  lcm_->subscribe("SANDIA_LEFT_STATE",&state_sync::leftHandHandler,this);  
  lcm_->subscribe("IROBOT_LEFT_STATE",&state_sync::leftHandHandler,this);  
  lcm_->subscribe("SANDIA_RIGHT_STATE",&state_sync::rightHandHandler,this);  
  lcm_->subscribe("IROBOT_RIGHT_STATE",&state_sync::rightHandHandler,this);  
  lcm_->subscribe("ATLAS_STATE",&state_sync::atlasHandler,this);  

  lcm_->subscribe("ATLAS_STATE_EXTRA",&state_sync::atlasExtraHandler,this);  
  lcm_->subscribe("ATLAS_POT_OFFSETS",&state_sync::potOffsetHandler,this);  
  
  lcm_->subscribe("POSE_BDI",&state_sync::poseBDIHandler,this); 
  pose_BDI_.utime =0; // use this to signify un-initalised
  

  // pot offsets if pots are used, currently uncalibrated
  pot_joint_offsets_.assign(28,0.0);

  // encoder offsets if encoders are used
  encoder_joint_offsets_.assign(28,0.0);
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_USY] = 0.008;
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_SHX] = 0.005;
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_ELY] = 3.1152;// -3.168 + 2*M_PI;
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_ELX] = -0.011;
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_UWY] = -1.085;
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_MWX] = 0.151;
  
  //maximum encoder angle before wrapping.  if q > max_angle, use q - 2*pi
  max_encoder_wrap_angle_.assign(28,0.0);
  max_encoder_wrap_angle_[Atlas::JOINT_R_ARM_ELY] = 2;
  max_encoder_wrap_angle_[Atlas::JOINT_L_ARM_ELY] = 2;

  use_encoder_.assign(28,false);
  use_encoder_[Atlas::JOINT_R_ARM_USY] = true;
  use_encoder_[Atlas::JOINT_R_ARM_SHX] = true;
  use_encoder_[Atlas::JOINT_R_ARM_ELY] = true;
  use_encoder_[Atlas::JOINT_R_ARM_ELX] = true;
  use_encoder_[Atlas::JOINT_R_ARM_UWY] = true;
  use_encoder_[Atlas::JOINT_R_ARM_MWX] = true;
 
}


void state_sync::potOffsetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg){
  std::cout << "got potOffsetHandler\n";
  pot_joint_offsets_ = msg->joint_position;
  
  for (size_t i=0; i < pot_joint_offsets_.size(); i++){
    std::cout << pot_joint_offsets_[i] << ", ";
  }
  std::cout << "\n";
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

void state_sync::leftHandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg){
  //std::cout << "got "<< channel <<"\n";
  left_hand_joints_.name = msg->joint_name;
  left_hand_joints_.position = msg->joint_position;
  left_hand_joints_.velocity = msg->joint_velocity;
  left_hand_joints_.effort = msg->joint_effort;
  
  if (standalone_hand_){ // assumes only one hand is actively publishing state
    drc::force_torque_t force_torque_msg;
    publishRobotState(msg->utime, force_torque_msg);
  }  
}

void state_sync::rightHandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg){
  //std::cout << "got "<< channel <<"\n";
  right_hand_joints_.name = msg->joint_name;
  right_hand_joints_.position = msg->joint_position;
  right_hand_joints_.velocity = msg->joint_velocity;
  right_hand_joints_.effort = msg->joint_effort;
  
  if (standalone_hand_){ // assumes only one hand is actively publishing state
    drc::force_torque_t force_torque_msg;
    publishRobotState(msg->utime, force_torque_msg);
  }    
}


void state_sync::atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg){
  //std::cout << "got atlasHandler\n";
  atlas_joints_.name = msg->joint_name;
  
  std::vector <float> mod_positions;
  mod_positions.assign(28,0.0);
  for (size_t i=0; i < pot_joint_offsets_.size(); i++){
    mod_positions[i] = msg->joint_position[i] + pot_joint_offsets_[i]; 
    ///std::cout << pot_joint_offsets_[i] << ", ";
  }  
  atlas_joints_.position = mod_positions;
  atlas_joints_.velocity = msg->joint_velocity;
  atlas_joints_.effort = msg->joint_effort;
  
  
  // Overwrite the actuator joint positions and velocities with the after-transmission 
  // sensor values for the ARMS ONLY (first exposed in v2.7.0 of BDI's API)
  // NB: this assumes that they are provided at the same rate as ATLAS_STATE
  if (use_transmission_joint_sensors_ ){
    if (atlas_joints_.position.size() == atlas_joints_out_.position.size()   ){
      if (atlas_joints_.velocity.size() == atlas_joints_out_.velocity.size()   ){
        // TOOD: determine the joint range of interest:
        for (int i=0; i < atlas_joints_out_.position.size() ; i++ ) { // apply the right arm only

          if (use_encoder_[i]) {
            atlas_joints_.position[i] = atlas_joints_out_.position[i];
            if (atlas_joints_.position[i] > max_encoder_wrap_angle_[i])
              atlas_joints_.position[i] -= 2*M_PI;

            atlas_joints_.position[i] += encoder_joint_offsets_[i];
            atlas_joints_.velocity[i] = atlas_joints_out_.velocity[i];
          }
        }
      }
    }
  }
  
  publishRobotState(msg->utime, msg->force_torque);
}

void state_sync::atlasExtraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_extra_t* msg){
  //std::cout << "got atlasExtraHandler\n";
  atlas_joints_out_.position = msg->joint_position_out;
  atlas_joints_out_.velocity = msg->joint_velocity_out;
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
  
  appendJoints(robot_state_msg, left_hand_joints_);
  appendJoints(robot_state_msg, right_hand_joints_);

  //std::cout << robot_state_msg.joint_name.size() << " Number of Joints\n";
  robot_state_msg.num_joints = robot_state_msg.joint_name.size();
  
  // Limb Sensor states
  robot_state_msg.force_torque = force_torque_msg;
  
  if (simulation_mode_){ // to be deprecated..
    lcm_->publish("TRUE_ROBOT_STATE", &robot_state_msg);    
  }
  
  if (bdi_motion_estimate_){
    if ( insertPoseBDI(robot_state_msg) ){
      lcm_->publish("EST_ROBOT_STATE", &robot_state_msg); 
    }
  }else if(standalone_head_ || standalone_hand_ ){
    lcm_->publish("EST_ROBOT_STATE", &robot_state_msg);
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
  bool standalone_hand = false;
  bool bdi_motion_estimate = false;
  bool simulation_mode = false;
  bool use_transmission_joint_sensors = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(standalone_head, "l", "standalone_head","Standalone Head");
  opt.add(standalone_hand, "f", "standalone_hand","Standalone Hand");
  opt.add(bdi_motion_estimate, "b", "bdi","Use POSE_BDI to make EST_ROBOT_STATE");
  opt.add(simulation_mode, "s", "simulation","Simulation mode - output TRUE RS");
  opt.add(use_transmission_joint_sensors, "t", "transmission","Use the transmission joint sensors (in the arms)");
  opt.parse();
  
  std::cout << "standalone_head: " << standalone_head << "\n";
  std::cout << "Use transmission joint sensors: " << use_transmission_joint_sensors << " (arms only)\n";

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  state_sync app(lcm, standalone_head,standalone_hand,bdi_motion_estimate, 
                 simulation_mode, use_transmission_joint_sensors);
  while(0 == lcm->handle());
  return 0;
}

