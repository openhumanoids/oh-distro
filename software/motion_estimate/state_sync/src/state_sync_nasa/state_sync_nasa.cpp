#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>
#include <vector>
#include <fstream>

#include "state_sync_nasa.hpp"
#include <ConciseArgs>
#include <sys/time.h>
#include <algorithm>


using namespace std;
#define DO_TIMING_PROFILE FALSE

/////////////////////////////////////
std::vector<std::string> jointNames;

void assignJointsStruct( Joints &joints ){
  joints.velocity.assign( joints.name.size(), 0);
  joints.position.assign( joints.name.size(), 0);
  joints.effort.assign( joints.name.size(), 0);  
}

void onParamChangeSync(BotParam* old_botparam, BotParam* new_botparam,
                     int64_t utime, void* user) {  
  state_sync_nasa& sync = *((state_sync_nasa*)user);
  sync.setBotParam(new_botparam);
}

state_sync_nasa::state_sync_nasa(boost::shared_ptr<lcm::LCM> &lcm_,
                       boost::shared_ptr<CommandLineConfig> &cl_cfg_):
                       lcm_(lcm_), cl_cfg_(cl_cfg_) {

  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  jointNames = model_->getJointNames();

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 1); // 1 means keep updated, 0 would ignore updates
  bot_param_add_update_subscriber(botparam_,
                                  onParamChangeSync, this);
   
  ////////////////////////////////////////////////////////////////////////////////////////
  /// 1. Get the Joint names and determine the correct configuration:
  
  core_robot_joints_.name = jointNames;
   
  assignJointsStruct( core_robot_joints_ );
  
  // The number of joints is expected to be 33; 32 robot joints (i.e. without the hands) plus the Hokuyo joint
  std::cout << "No. of Joints: "
      << core_robot_joints_.position.size() << " valkyrie\n";

  // Output list of all joints
  for (unsigned int i = 0; i < core_robot_joints_.position.size(); i++) {
    std::cout << core_robot_joints_.name[i] << " is joint no. " << i << std::endl;
  }

  /// 2. Subscribe to required signals
  lcm::Subscription* sub0;
  lcm::Subscription* sub1;
  if (cl_cfg_->mode == "ihmc") {
    std::cout << "Using IHMC as source for robot state and force torque" << std::endl;
    sub0 = lcm_->subscribe("CORE_ROBOT_STATE", &state_sync_nasa::coreRobotHandler, this);
    sub1 = lcm_->subscribe("FORCE_TORQUE", &state_sync_nasa::forceTorqueHandler, this);
  } else if (cl_cfg_->mode == "nasa") {
    std::cout << "Using NASA as source for robot state and force torque" << std::endl;
    sub0 = lcm_->subscribe("VAL_CORE_ROBOT_STATE", &state_sync_nasa::coreRobotHandler, this);
    sub1 = lcm_->subscribe("VAL_FORCE_TORQUE", &state_sync_nasa::forceTorqueHandler, this);
  } else {
    std::cerr << "Unsupported mode! Must be either nasa or ihmc" << std::endl;
  }
  force_torque_init_ = false;
  ///////////////////////////////////////////////////////////////
  lcm::Subscription* sub2 = lcm_->subscribe("POSE_BDI",&state_sync_nasa::poseIHMCHandler,this); // Always provided by the IHMC Driver:
  lcm::Subscription* sub3 = lcm_->subscribe("POSE_BDI",&state_sync_nasa::poseProntoHandler,this);  // Always provided the state estimator:
  lcm::Subscription* sub4 = lcm_->subscribe("NECK_STATE",&state_sync_nasa::neckStateHandler,this);  // Provided when NeckController is running

  
  bool use_short_queue = true;
  if (use_short_queue){
    sub0->setQueueCapacity(1);
    sub1->setQueueCapacity(1);
    sub2->setQueueCapacity(1);
    sub3->setQueueCapacity(1);
    sub4->setQueueCapacity(1);
  }

  setPoseToZero(pose_IHMC_);
  setPoseToZero(pose_Pronto_);


  /// 4. Joint Filtering
  cl_cfg_->use_torque_adjustment = bot_param_get_boolean_or_fail(botparam_, "state_estimator.legodo.torque_adjustment" );
  if (cl_cfg_->use_torque_adjustment){
    std::cout << "Torque-based joint angle adjustment: Using\n";

    int n_gains = bot_param_get_array_len (botparam_, "state_estimator.legodo.adjustment_gain");
    double gains_in[n_gains];
    bot_param_get_double_array_or_fail(botparam_, "state_estimator.legodo.adjustment_gain", &gains_in[0], n_gains);
    std::vector<float> k;
    for (int i =0; i < n_gains;i++){
      k.push_back( (float) gains_in[i] );
    }
    torque_adjustment_ = new EstimateTools::TorqueAdjustment(k);

  }else{
    std::cout << "Torque-based joint angle adjustment: Not Using\n";
  }

}

void state_sync_nasa::setPoseToZero(PoseT &pose){
  pose.utime = 0; // use this to signify un-initalised
  pose.pos << 0,0, 0.95;// for ease of use//Eigen::Vector3d::Identity();
  pose.vel << 0,0,0;
  pose.orientation << 1.,0.,0.,0.;
  pose.rotation_rate << 0,0,0;
  pose.accel << 0,0,0;
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

void state_sync_nasa::forceTorqueHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::six_axis_force_torque_array_t* msg){
  force_torque_ = *msg;
  force_torque_init_ = true; 
}

void state_sync_nasa::coreRobotHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg){
  if (!force_torque_init_){
    std::cout << "FORCE_TORQUE not received yet, not publishing EST_ROBOT_STATE =========================\n";
    return;    
  }   

  //checkJointLengths(core_robot_joints_.position.size(),  msg->joint_position.size(), channel);

  std::vector <float> mod_positions;
  mod_positions = msg->joint_position;
  core_robot_joints_.position = mod_positions;

  core_robot_joints_.name = msg->joint_name;  // added recently
  core_robot_joints_.velocity = msg->joint_velocity;
  core_robot_joints_.effort = msg->joint_effort;
  
  for (int i=0; i<jointNames.size(); i++) {
    if (std::find(core_robot_joints_.name.begin(), core_robot_joints_.name.end(), jointNames[i]) == core_robot_joints_.name.end()) {
      mod_positions.push_back(0);
      core_robot_joints_.name.push_back(jointNames[i]);
      core_robot_joints_.position.push_back(0);
      core_robot_joints_.velocity.push_back(0);
      core_robot_joints_.effort.push_back(0);
    }
  }

  if (cl_cfg_->use_torque_adjustment){
    torque_adjustment_->processSample(core_robot_joints_.position, core_robot_joints_.effort );
  }

  // TODO: check forque_
  publishRobotState(msg->utime, force_torque_);
}

bot_core::rigid_transform_t getIsometry3dAsBotRigidTransform(Eigen::Isometry3d pose, int64_t utime){
  bot_core::rigid_transform_t tf;
  tf.utime = utime;
  tf.trans[0] = pose.translation().x();
  tf.trans[1] = pose.translation().y();
  tf.trans[2] = pose.translation().z();
  Eigen::Quaterniond quat(pose.rotation());
  tf.quat[0] = quat.w();
  tf.quat[1] = quat.x();
  tf.quat[2] = quat.y();
  tf.quat[3] = quat.z();
  return tf;

}

// Handle message published by JointPositionGoalController (Neck Controller)
void state_sync_nasa::neckStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg) {
  for (int i=0; i < msg->num_joints; i++) {
    auto it = std::find(core_robot_joints_.name.begin(), core_robot_joints_.name.end(), msg->joint_name[i]);
    int joint_index = std::distance(core_robot_joints_.name.begin(), it);

    core_robot_joints_.position[joint_index] = msg->joint_position[i];
    core_robot_joints_.velocity[joint_index] = 0;
    core_robot_joints_.effort[joint_index] = 0;
  }
}

Eigen::Isometry3d getPoseAsIsometry3d(PoseT pose){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();
  pose_iso.translation()  << pose.pos[0], pose.pos[1] , pose.pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(pose.orientation[0], pose.orientation[1], 
                                               pose.orientation[2], pose.orientation[3]);
  pose_iso.rotate(quat);
  return pose_iso;
}

void state_sync_nasa::poseIHMCHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  pose_IHMC_.utime = msg->utime;
  pose_IHMC_.pos = Eigen::Vector3d( msg->pos[0],  msg->pos[1],  msg->pos[2] );
  pose_IHMC_.vel = Eigen::Vector3d( msg->vel[0],  msg->vel[1],  msg->vel[2] );
  pose_IHMC_.orientation = Eigen::Vector4d( msg->orientation[0],  msg->orientation[1],  msg->orientation[2],  msg->orientation[3] );
  pose_IHMC_.rotation_rate = Eigen::Vector3d( msg->rotation_rate[0],  msg->rotation_rate[1],  msg->rotation_rate[2] );
  pose_IHMC_.accel = Eigen::Vector3d( msg->accel[0],  msg->accel[1],  msg->accel[2] );
}

void state_sync_nasa::poseProntoHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  pose_Pronto_.utime = msg->utime;
  pose_Pronto_.pos = Eigen::Vector3d( msg->pos[0],  msg->pos[1],  msg->pos[2] );
  pose_Pronto_.vel = Eigen::Vector3d( msg->vel[0],  msg->vel[1],  msg->vel[2] );
  pose_Pronto_.orientation = Eigen::Vector4d( msg->orientation[0],  msg->orientation[1],  msg->orientation[2],  msg->orientation[3] );
  pose_Pronto_.rotation_rate = Eigen::Vector3d( msg->rotation_rate[0],  msg->rotation_rate[1],  msg->rotation_rate[2] );
  pose_Pronto_.accel = Eigen::Vector3d( msg->accel[0],  msg->accel[1],  msg->accel[2] );

  // If State sync has received POSE_BDI and POSE_BODY, we must be running our own estimator
  // So there will be a difference between these, so publish this for things like walking footstep transformations
  // TODO: rate limit this to something like 10Hz
  // TODO: this might need to be published only when pose_BDI_.utime and pose_MIT_.utime are very similar
  if ( pose_IHMC_.utime > 0 ){
    Eigen::Isometry3d localmit_to_bodymit = getPoseAsIsometry3d(pose_Pronto_);
    Eigen::Isometry3d localmit_to_bodybdi = getPoseAsIsometry3d(pose_IHMC_);
// localmit_to_body.inverse() * localmit_to_body_bdi;
    // Eigen::Isometry3d localmit_to_localbdi = localmit_to_bodymit.inverse() * localmit_to_bodymit.inverse() * localmit_to_bodybdi;
    Eigen::Isometry3d localmit_to_localbdi = localmit_to_bodybdi * localmit_to_bodymit.inverse();

    bot_core::rigid_transform_t localmit_to_localbdi_msg = getIsometry3dAsBotRigidTransform( localmit_to_localbdi, pose_Pronto_.utime );
    lcm_->publish("LOCAL_TO_LOCAL_BDI", &localmit_to_localbdi_msg);    
  }

}

// Returns false if the pose is old or hasn't appeared yet
bool state_sync_nasa::insertPoseInRobotState(bot_core::robot_state_t& msg, PoseT pose){
  
  msg.pose.translation.x = pose.pos[0];
  msg.pose.translation.y = pose.pos[1];
  msg.pose.translation.z = pose.pos[2];
  msg.pose.rotation.w = pose.orientation[0];
  msg.pose.rotation.x = pose.orientation[1];
  msg.pose.rotation.y = pose.orientation[2];
  msg.pose.rotation.z = pose.orientation[3];

  // Both incoming velocities (from PoseT) are assumed to be in body frame, 
  // convention is for EST_ROBOT_STATE to be in local frame
  // convert here:
  Eigen::Matrix3d R = Eigen::Matrix3d( Eigen::Quaterniond( pose.orientation[0], pose.orientation[1], pose.orientation[2],pose.orientation[3] ));
  Eigen::Vector3d lin_vel_local = R*Eigen::Vector3d ( pose.vel[0], pose.vel[1], pose.vel[2]);
  msg.twist.linear_velocity.x = lin_vel_local[0];
  msg.twist.linear_velocity.y = lin_vel_local[1];
  msg.twist.linear_velocity.z = lin_vel_local[2];

  Eigen::VectorXd rr_prefiltered(3);
  rr_prefiltered << pose.rotation_rate[0], pose.rotation_rate[1], pose.rotation_rate[2] ;

  Eigen::Vector3d rot_vel_local = R*Eigen::Vector3d ( rr_prefiltered[0], rr_prefiltered[1], rr_prefiltered[2] );
  msg.twist.angular_velocity.x = rot_vel_local[0];
  msg.twist.angular_velocity.y = rot_vel_local[1];
  msg.twist.angular_velocity.z = rot_vel_local[2];
  
  return true;  
}

void state_sync_nasa::publishRobotState(int64_t utime_in,  const  bot_core::six_axis_force_torque_array_t& force_torque_msg){
  
  bot_core::robot_state_t robot_state_msg;
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
  appendJoints(robot_state_msg, core_robot_joints_);  
  
  //std::cout << robot_state_msg.joint_name.size() << " Number of Joints\n";
  robot_state_msg.num_joints = robot_state_msg.joint_name.size();
  
  // Limb Sensor states
  bot_core::force_torque_t force_torque_convert;
  if (force_torque_msg.sensors.size() == 4) {
    force_torque_convert.l_foot_force_z = force_torque_msg.sensors[0].force[2];
    force_torque_convert.l_foot_torque_x = force_torque_msg.sensors[0].moment[0];
    force_torque_convert.l_foot_torque_y = force_torque_msg.sensors[0].moment[1];

    force_torque_convert.r_foot_force_z = force_torque_msg.sensors[1].force[2];
    force_torque_convert.r_foot_torque_x = force_torque_msg.sensors[1].moment[0];
    force_torque_convert.r_foot_torque_y = force_torque_msg.sensors[1].moment[1];

    force_torque_convert.l_hand_force[0] = force_torque_msg.sensors[2].force[0];
    force_torque_convert.l_hand_force[1] = force_torque_msg.sensors[2].force[1];
    force_torque_convert.l_hand_force[2] = force_torque_msg.sensors[2].force[2];
    force_torque_convert.l_hand_torque[0] = force_torque_msg.sensors[2].moment[0];
    force_torque_convert.l_hand_torque[1] = force_torque_msg.sensors[2].moment[1];
    force_torque_convert.l_hand_torque[2] = force_torque_msg.sensors[2].moment[2];

    force_torque_convert.r_hand_force[0] = force_torque_msg.sensors[3].force[0];
    force_torque_convert.r_hand_force[1] = force_torque_msg.sensors[3].force[1];
    force_torque_convert.r_hand_force[2] = force_torque_msg.sensors[3].force[2];
    force_torque_convert.r_hand_torque[0] = force_torque_msg.sensors[3].moment[0];
    force_torque_convert.r_hand_torque[1] = force_torque_msg.sensors[3].moment[1];
    force_torque_convert.r_hand_torque[2] = force_torque_msg.sensors[3].moment[2];
  }

  robot_state_msg.force_torque = force_torque_convert;
  
  if ( insertPoseInRobotState(robot_state_msg, pose_Pronto_) ){
    lcm_->publish( cl_cfg_->output_channel, &robot_state_msg);
  }
}

void state_sync_nasa::appendJoints(bot_core::robot_state_t& msg_out, Joints joints){
  for (size_t i = 0; i < joints.position.size(); i++)  {
    msg_out.joint_name.push_back( joints.name[i] );
    msg_out.joint_position.push_back( joints.position[i] );
    msg_out.joint_velocity.push_back( joints.velocity[i]);
    msg_out.joint_effort.push_back( joints.effort[i] );
  }
}

int
main(int argc, char ** argv){
  boost::shared_ptr<CommandLineConfig> cl_cfg(new CommandLineConfig() );  
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg->output_channel, "o", "output_channel","Output Channel for robot state msg");
  opt.add(cl_cfg->mode, "m", "mode","Mode - ihmc or nasa (source of joint states and force torque messages)");
  opt.parse();
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  state_sync_nasa app(lcm, cl_cfg);
  while(0 == lcm->handle());
  return 0;
}
