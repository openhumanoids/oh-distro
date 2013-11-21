#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>
#include <vector>

#include "state_sync.hpp"
#include <ConciseArgs>

using namespace std;
#define DO_TIMING_PROFILE FALSE

/////////////////////////////////////

void assignJointsStruct( Joints &joints ){
  joints.velocity.assign( joints.name.size(), 0);
  joints.position.assign( joints.name.size(), 0);
  joints.effort.assign( joints.name.size(), 0);
  
}


state_sync::state_sync(boost::shared_ptr<lcm::LCM> &lcm_, 
                       bool standalone_head_, bool standalone_hand_,  
                       bool bdi_motion_estimate_, bool simulation_mode_,
                       bool use_encoder_joint_sensors_):
   lcm_(lcm_), 
   standalone_head_(standalone_head_), standalone_hand_(standalone_hand_),
   bdi_motion_estimate_(bdi_motion_estimate_), simulation_mode_(simulation_mode_),
   use_encoder_joint_sensors_(use_encoder_joint_sensors_){
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));     

  // Get the Joint names and determine the correct configuration:
  std::vector<std::string> joint_names = model_->getJointNames();
  
  if(find(joint_names.begin(), joint_names.end(), "left_f0_j0" ) != joint_names.end()){
    std::cout << "Robot fitted with left Sandia hand\n";
    lcm_->subscribe("SANDIA_LEFT_STATE",&state_sync::leftHandHandler,this);  
    left_hand_joints_.name = joint_utils_.sandia_l_joint_names;
  }else if(find(joint_names.begin(), joint_names.end(), "left_finger[0]/joint_base" ) != joint_names.end()){
    std::cout << "Robot fitted with left iRobot hand\n";
    lcm_->subscribe("IROBOT_LEFT_STATE",&state_sync::leftHandHandler,this);  
    left_hand_joints_.name = joint_utils_.irobot_l_joint_names;
  }else{
    std::cout << "Robot has no left hand\n"; 
  }

  if(find(joint_names.begin(), joint_names.end(), "right_f0_j0" ) != joint_names.end()){
    std::cout << "Robot fitted with right Sandia hand\n";
    lcm_->subscribe("SANDIA_RIGHT_STATE",&state_sync::rightHandHandler,this);
    right_hand_joints_.name = joint_utils_.sandia_r_joint_names;
  }else if(find(joint_names.begin(), joint_names.end(), "right_finger[0]/joint_base" ) != joint_names.end()){
    std::cout << "Robot fitted with right iRobot hand\n";
    lcm_->subscribe("IROBOT_RIGHT_STATE",&state_sync::rightHandHandler,this);
    right_hand_joints_.name = joint_utils_.irobot_r_joint_names;
  }else{
    std::cout << "Robot has no right hand\n"; 
  }
  
  atlas_joints_.name = joint_utils_.atlas_joint_names; 
  
  if(find(joint_names.begin(), joint_names.end(), "pre_spindle_cal_x_joint" ) != joint_names.end()){
    std::cout << "Robot fitted with dummy head joints\n";
    head_joints_.name = joint_utils_.head_joint_names;
  }else{
    std::cout << "Robot fitted with a single head joint\n";
    head_joints_.name = joint_utils_.simple_head_joint_names;
  }
  lcm_->subscribe("MULTISENSE_STATE",&state_sync::multisenseHandler,this);
  
  
  assignJointsStruct( atlas_joints_ );
  assignJointsStruct( head_joints_ );
  assignJointsStruct( left_hand_joints_ );
  assignJointsStruct( right_hand_joints_ );
  
  std::cout << "No. of Joints: "
      << atlas_joints_.position.size() << " atlas, "
      << head_joints_.position.size() << " head, "
      << left_hand_joints_.position.size() << " left, "
      << right_hand_joints_.position.size() << " right\n";
  
  lcm_->subscribe("ATLAS_STATE",&state_sync::atlasHandler,this);  

  ///////////////////////////////////////////////////////////////
  lcm_->subscribe("ATLAS_STATE_EXTRA",&state_sync::atlasExtraHandler,this);  
  lcm_->subscribe("ATLAS_POT_OFFSETS",&state_sync::potOffsetHandler,this);  
  
  lcm_->subscribe("POSE_BDI",&state_sync::poseBDIHandler,this); 
  pose_BDI_.utime =0; // use this to signify un-initalised
  
  
  /// Pots and Encoders:
  // pot offsets if pots are used, currently uncalibrated
  pot_joint_offsets_.assign(28,0.0);

  // encoder offsets if encoders are used
  encoder_joint_offsets_.assign(28,0.0);
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_USY] = 3.14; // robot software v1.9
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_SHX] = -0.02; // robot software v1.9
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_ELY] = -0.15; // robot software v1.9
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_ELX] = 0.02; // robot software v1.9
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_UWY] = 2.055; // robot software v1.9
  encoder_joint_offsets_[Atlas::JOINT_R_ARM_MWX] = 0.0; // robot software v1.9
  
  // encoder_joint_offsets_[Atlas::JOINT_R_ARM_USY] = 0.0182; // robot software v1.8
  // encoder_joint_offsets_[Atlas::JOINT_R_ARM_SHX] = 0.0052; // robot software v1.8
  // encoder_joint_offsets_[Atlas::JOINT_R_ARM_ELY] = -0.0130; // robot software v1.8
  // encoder_joint_offsets_[Atlas::JOINT_R_ARM_ELX] = 0.0361; // robot software v1.8
  // encoder_joint_offsets_[Atlas::JOINT_R_ARM_UWY] = -1.0802; // robot software v1.8
  // encoder_joint_offsets_[Atlas::JOINT_R_ARM_MWX] = 0.0104; // robot software v1.8

  encoder_joint_offsets_[Atlas::JOINT_L_ARM_USY] = -0.027; // robot software v1.8/9
  encoder_joint_offsets_[Atlas::JOINT_L_ARM_SHX] = -0.0201; // robot software v1.8/9
  encoder_joint_offsets_[Atlas::JOINT_L_ARM_ELY] = 3.13; // robot software v1.8/9
  encoder_joint_offsets_[Atlas::JOINT_L_ARM_ELX] = -0.0202; // robot software v1.8/9
  encoder_joint_offsets_[Atlas::JOINT_L_ARM_UWY] = -0.0114; // robot software v1.8/9
  encoder_joint_offsets_[Atlas::JOINT_L_ARM_MWX] = 0.0290; // robot software v1.8/9

  //encoder_joint_offsets_[Atlas::JOINT_NECK_AY] = 1.1125; // robot software v1.8
  encoder_joint_offsets_[Atlas::JOINT_NECK_AY] = 4.235 - 2*M_PI;  // robot software v1.9

  //maximum encoder angle before wrapping.  if q > max_angle, use q - 2*pi
  // if q < min_angle, use q + 2*pi
  max_encoder_wrap_angle_.assign(28,100000000);
  max_encoder_wrap_angle_[Atlas::JOINT_R_ARM_UWY] = 4; // robot software v1.9
  max_encoder_wrap_angle_[Atlas::JOINT_R_ARM_USY] = 0; // robot software v1.9
  max_encoder_wrap_angle_[Atlas::JOINT_L_ARM_ELY] = 3; // robot software v1.9

  use_encoder_.assign(28,false);
  use_encoder_[Atlas::JOINT_R_ARM_USY] = true;
  use_encoder_[Atlas::JOINT_R_ARM_SHX] = true;
  use_encoder_[Atlas::JOINT_R_ARM_ELY] = true;
  use_encoder_[Atlas::JOINT_R_ARM_ELX] = true;
  use_encoder_[Atlas::JOINT_R_ARM_UWY] = true;
  use_encoder_[Atlas::JOINT_R_ARM_MWX] = true;

  use_encoder_[Atlas::JOINT_L_ARM_USY] = true;
  use_encoder_[Atlas::JOINT_L_ARM_SHX] = true;
  use_encoder_[Atlas::JOINT_L_ARM_ELY] = true;
  use_encoder_[Atlas::JOINT_L_ARM_ELX] = true;
  use_encoder_[Atlas::JOINT_L_ARM_UWY] = true;
  use_encoder_[Atlas::JOINT_L_ARM_MWX] = true;

  use_encoder_[Atlas::JOINT_NECK_AY] = true;
}


void state_sync::potOffsetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg){
  std::cout << "got potOffsetHandler\n";
  pot_joint_offsets_ = msg->joint_position;
  
  for (size_t i=0; i < pot_joint_offsets_.size(); i++){
    std::cout << pot_joint_offsets_[i] << ", ";
  }
  std::cout << "\n";
}

// Quick check that the incoming and previous joint sets are the same size
// TODO: perhaps make this more careful with more checks?
void checkJointLengths(size_t previous_size , size_t incoming_size, std::string channel){
  if ( incoming_size != previous_size ){
    std::cout << "ERROR: Number of joints in " << channel << "[" << incoming_size 
              << "] does not match previous [" << previous_size << "]\n"; 
    exit(-1);
  }  
}


void state_sync::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg){
  checkJointLengths( head_joints_.position.size(),  msg->joint_position.size(), channel);
  
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
  checkJointLengths( left_hand_joints_.position.size(),  msg->joint_position.size(), channel);
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
  checkJointLengths( right_hand_joints_.position.size(),  msg->joint_position.size(), channel);
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
  checkJointLengths( atlas_joints_.position.size(),  msg->joint_position.size(), channel);
  

  std::vector <float> mod_positions;
  mod_positions.assign(28,0.0);
  for (size_t i=0; i < pot_joint_offsets_.size(); i++){
    mod_positions[i] = msg->joint_position[i] + pot_joint_offsets_[i]; 
    ///std::cout << pot_joint_offsets_[i] << ", ";
  }  
  atlas_joints_.position = mod_positions;
  atlas_joints_.velocity = msg->joint_velocity;
  atlas_joints_.effort = msg->joint_effort;
  // atlas_joints_.name = atlas_joint_names_;
  
  
  // Overwrite the actuator joint positions and velocities with the after-transmission 
  // sensor values for the ARMS ONLY (first exposed in v2.7.0 of BDI's API)
  // NB: this assumes that they are provided at the same rate as ATLAS_STATE
  if (use_encoder_joint_sensors_ ){
    if (atlas_joints_.position.size() == atlas_joints_out_.position.size()   ){
      if (atlas_joints_.velocity.size() == atlas_joints_out_.velocity.size()   ){
        for (int i=0; i < atlas_joints_out_.position.size() ; i++ ) { 

          if (use_encoder_[i]) {
            atlas_joints_.position[i] = atlas_joints_out_.position[i];
            if (atlas_joints_.position[i] > max_encoder_wrap_angle_[i])
              atlas_joints_.position[i] -= 2*M_PI;
            atlas_joints_.position[i] += encoder_joint_offsets_[i];

            // check for wonky encoder initialization :(
            while (atlas_joints_.position[i] - mod_positions[i] > 0.5)
              atlas_joints_.position[i] -= 2*M_PI/3;
            while (atlas_joints_.position[i] - mod_positions[i] < -0.5)
              atlas_joints_.position[i] += 2*M_PI/3;

            atlas_joints_.velocity[i] = atlas_joints_out_.velocity[i];

            // copy pot positions back into _out so we have them in the lcm log
            atlas_joints_out_.position[i] = mod_positions[i];
            atlas_joints_out_.position[i] = msg->joint_velocity[i];
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
  if (pose_BDI_.utime ==0){
    std::cout << "haven't received POSE_BDI, refusing to publish ERS\n";
    return false;
  }
  
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
  
  // std::cout << "sending " << robot_state_msg.num_joints << " joints\n";

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
  bool use_encoder_joint_sensors = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(standalone_head, "l", "standalone_head","Standalone Head");
  opt.add(standalone_hand, "f", "standalone_hand","Standalone Hand");
  opt.add(bdi_motion_estimate, "b", "bdi","Use POSE_BDI to make EST_ROBOT_STATE");
  opt.add(simulation_mode, "s", "simulation","Simulation mode - output TRUE RS");
  opt.add(use_encoder_joint_sensors, "e", "encoder","Use the encoder joint sensors (in the arms)");
  opt.parse();
  
  std::cout << "standalone_head: " << standalone_head << "\n";
  std::cout << "Use transmission joint sensors: " << use_encoder_joint_sensors << " (arms only)\n";

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  state_sync app(lcm, standalone_head,standalone_hand,bdi_motion_estimate, 
                 simulation_mode, use_encoder_joint_sensors);
  while(0 == lcm->handle());
  return 0;
}

