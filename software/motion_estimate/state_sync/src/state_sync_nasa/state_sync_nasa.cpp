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

inline double clamp(double x, double lower, double upper) {
  return std::max(lower, std::min(upper, x));
}
inline double toRad(double deg) { return (deg * M_PI / 180); }

state_sync_nasa::state_sync_nasa(std::shared_ptr<lcm::LCM> &lcm_,
                       std::shared_ptr<CommandLineConfig> &cl_cfg_):
                       lcm_(lcm_), cl_cfg_(cl_cfg_) {
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 1); // 1 means keep updated, 0 would ignore updates
  bot_param_add_update_subscriber(botparam_,
                                  onParamChangeSync, this);

  std::shared_ptr<ModelClient> model_client = std::shared_ptr<ModelClient>(
      new ModelClient(lcm_->getUnderlyingLCM(), 0));
  RigidBodyTree model;
  model.addRobotFromURDFString(model_client->getURDFString());
  model.compile();
  std::map<std::string, int> dof_map = model.computePositionNameToIndexMap();

  for (auto iter = dof_map.begin(); iter != dof_map.end(); ++iter) {
    // std::cout << iter->first << " has min "
    //   << model.joint_limit_min[iter->second]
    //   << " and max "
    //   << model.joint_limit_max[iter->second] << std::endl;

    joint_limit_min_.insert(std::pair<std::string, double>(
        iter->first, model.joint_limit_min[iter->second]));
    joint_limit_max_.insert(std::pair<std::string, double>(
        iter->first, model.joint_limit_max[iter->second]));
  }

  // Subscribe to required signals
  lcm::Subscription* sub0 = lcm_->subscribe("CORE_ROBOT_STATE", &state_sync_nasa::coreRobotHandler, this);
  lcm::Subscription* sub1 = lcm_->subscribe("FORCE_TORQUE", &state_sync_nasa::forceTorqueHandler, this);
  lcm::Subscription* sub2 = lcm_->subscribe("POSE_BODY_ALT",&state_sync_nasa::poseIHMCHandler,this); // Always provided by the IHMC Driver
  lcm::Subscription* sub3;
  if (cl_cfg_->use_ihmc){
    sub3 = lcm_->subscribe("POSE_BODY_ALT",&state_sync_nasa::poseBodyHandler,this);  // If the Pronto state estimator isn't running
  }else{
    sub3 = lcm_->subscribe("POSE_BODY",&state_sync_nasa::poseBodyHandler,this);  // Always provided the state estimator:
  }
  lcm::Subscription* sub4 = lcm_->subscribe("NECK_STATE",&state_sync_nasa::neckStateHandler,this);  // Provided when NeckController is running
  force_torque_init_ = false;

  
  bool use_short_queue = true;
  if (use_short_queue){
    sub0->setQueueCapacity(1);
    sub1->setQueueCapacity(1);
    sub2->setQueueCapacity(1);
    sub3->setQueueCapacity(1);
    sub4->setQueueCapacity(1);
  }

  setPoseToZero(pose_ihmc_);
  setPoseToZero(pose_pronto_);


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

    std::vector<std::string> jnames_v;
    char** jnames = bot_param_get_str_array_alloc(botparam_, "state_estimator.legodo.adjustment_joints");
    if (jnames == NULL) {
      fprintf(stderr, "Error: must specify state_estimator.legodo.adjustment_joints\n");
      exit(1);
    }
    else {
      for (int i = 0; jnames[i]; i++) {
        jnames_v.push_back(std::string(jnames[i]));
      }
    }
    bot_param_str_array_free(jnames);

    torque_adjustment_ = new EstimateTools::TorqueAdjustment(jnames_v, k);
  }else{
    std::cout << "Torque-based joint angle adjustment: Not Using\n";
  }

}

void state_sync_nasa::setPoseToZero(PoseT &pose){
  pose.utime = 0; // use this to signify un-initalised
  pose.pos << 0,0, 0.95; // for ease of use, initialise at typical height
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
  core_robot_joints_.position = msg->joint_position;

  core_robot_joints_.name = msg->joint_name;  // added recently
  core_robot_joints_.velocity = msg->joint_velocity;
  core_robot_joints_.effort = msg->joint_effort;
  

  if (cl_cfg_->use_torque_adjustment){
    torque_adjustment_->processSample(core_robot_joints_.name, core_robot_joints_.position, core_robot_joints_.effort );
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
  pose_ihmc_.utime = msg->utime;
  pose_ihmc_.pos = Eigen::Vector3d( msg->pos[0],  msg->pos[1],  msg->pos[2] );
  pose_ihmc_.vel = Eigen::Vector3d( msg->vel[0],  msg->vel[1],  msg->vel[2] );
  pose_ihmc_.orientation = Eigen::Vector4d( msg->orientation[0],  msg->orientation[1],  msg->orientation[2],  msg->orientation[3] );
  pose_ihmc_.rotation_rate = Eigen::Vector3d( msg->rotation_rate[0],  msg->rotation_rate[1],  msg->rotation_rate[2] );
  pose_ihmc_.accel = Eigen::Vector3d( msg->accel[0],  msg->accel[1],  msg->accel[2] );
}

void state_sync_nasa::poseBodyHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  pose_pronto_.utime = msg->utime;
  pose_pronto_.pos = Eigen::Vector3d( msg->pos[0],  msg->pos[1],  msg->pos[2] );
  pose_pronto_.vel = Eigen::Vector3d( msg->vel[0],  msg->vel[1],  msg->vel[2] );
  pose_pronto_.orientation = Eigen::Vector4d( msg->orientation[0],  msg->orientation[1],  msg->orientation[2],  msg->orientation[3] );
  pose_pronto_.rotation_rate = Eigen::Vector3d( msg->rotation_rate[0],  msg->rotation_rate[1],  msg->rotation_rate[2] );
  pose_pronto_.accel = Eigen::Vector3d( msg->accel[0],  msg->accel[1],  msg->accel[2] );


  // pin the floating base if we set that option
  if(cl_cfg_->pin_floating_base){
    pose_pronto_.pos = Eigen::Vector3d(0,0,1);
  }

  // If State sync has received POSE_BDI and POSE_BODY, we must be running our own estimator
  // So there will be a difference between these, so publish this for things like walking footstep transformations
  // TODO: rate limit this to something like 10Hz
  // TODO: this might need to be published only when pose_BDI_.utime and pose_MIT_.utime are very similar
  if ( pose_ihmc_.utime > 0 ){
    Eigen::Isometry3d localmit_to_bodymit = getPoseAsIsometry3d(pose_pronto_);
    Eigen::Isometry3d localmit_to_bodybdi = getPoseAsIsometry3d(pose_ihmc_);
// localmit_to_body.inverse() * localmit_to_body_bdi;
    // Eigen::Isometry3d localmit_to_localbdi = localmit_to_bodymit.inverse() * localmit_to_bodymit.inverse() * localmit_to_bodybdi;
    Eigen::Isometry3d localmit_to_localbdi = localmit_to_bodybdi * localmit_to_bodymit.inverse();

    bot_core::rigid_transform_t localmit_to_localbdi_msg = getIsometry3dAsBotRigidTransform( localmit_to_localbdi, pose_pronto_.utime );
    lcm_->publish("LOCAL_TO_LOCAL_ALT", &localmit_to_localbdi_msg);    
  }

  // If using the IHMC estimate, then publish it to the rest of the system as POSE_BODY
  if (cl_cfg_->use_ihmc){
     lcm_->publish("POSE_BODY",msg);

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
  if (force_torque_msg.sensors.size() >= 2) {
    force_torque_convert.l_foot_force_z = force_torque_msg.sensors[0].force[2];
    force_torque_convert.l_foot_torque_x = force_torque_msg.sensors[0].moment[0];
    force_torque_convert.l_foot_torque_y = force_torque_msg.sensors[0].moment[1];

    force_torque_convert.r_foot_force_z = force_torque_msg.sensors[1].force[2];
    force_torque_convert.r_foot_torque_x = force_torque_msg.sensors[1].moment[0];
    force_torque_convert.r_foot_torque_y = force_torque_msg.sensors[1].moment[1];
  }

  if (force_torque_msg.sensors.size() == 4) {
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
  
  if ( insertPoseInRobotState(robot_state_msg, pose_pronto_) ){
    lcm_->publish( cl_cfg_->output_channel, &robot_state_msg);
  }
}

void state_sync_nasa::appendJoints(bot_core::robot_state_t& msg_out,
                                   Joints joints) {
  for (size_t i = 0; i < joints.position.size(); i++) {
    msg_out.joint_name.push_back(joints.name[i]);

    // Check whether joint is to be clamped to its joint limits
    msg_out.joint_position.push_back(
        clampJointToJointLimits(joints.name[i], joints.position[i]));

    msg_out.joint_velocity.push_back(joints.velocity[i]);
    msg_out.joint_effort.push_back(joints.effort[i]);
  }
}

float state_sync_nasa::clampJointToJointLimits(std::string joint_name,
                                               float joint_position) {
  if (std::find(joints_to_be_clamped_to_joint_limits_.begin(),
                joints_to_be_clamped_to_joint_limits_.end(),
                joint_name) != joints_to_be_clamped_to_joint_limits_.end()) {
    // Clamp if within +/-clamping_tolerance_in_degrees_, else return
    // warning/error and raw value
    double tolerance = toRad(clamping_tolerance_in_degrees_);
    double& lower_limit = joint_limit_min_[joint_name];
    double& upper_limit = joint_limit_max_[joint_name];

    if ((joint_position + lower_limit) > tolerance ||
        (joint_position - upper_limit) > tolerance) {
      std::cerr << "WARNING: " << joint_name << " deviates >"
                << clamping_tolerance_in_degrees_
                << "deg from joint limits, not clamping" << std::endl;
      return joint_position;
    } else {
      return clamp(joint_position, lower_limit, upper_limit);
    }
  } else {
    return joint_position;
  }
}

int
main(int argc, char ** argv){
  std::shared_ptr<CommandLineConfig> cl_cfg(new CommandLineConfig());
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg->output_channel, "o", "output_channel","Output Channel for robot state msg");
  opt.add(cl_cfg->use_ihmc, "i", "use_ihmc","Use the IHMC estimate of the body frame in ERS");
  opt.add(cl_cfg->pin_floating_base, "pinned", "pinned_floating_base", "pin floating base position");
  opt.parse();

  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  if (!lcm->good()) return 1;

  // Add joints which are OK to be clamped to their respective joint limits
  std::vector<std::string> joints_to_be_clamped_to_joint_limits;
  joints_to_be_clamped_to_joint_limits.push_back("lowerNeckPitch");
  joints_to_be_clamped_to_joint_limits.push_back("upperNeckPitch");
  joints_to_be_clamped_to_joint_limits.push_back("leftWristRoll");
  joints_to_be_clamped_to_joint_limits.push_back("leftWristPitch");
  joints_to_be_clamped_to_joint_limits.push_back("rightWristRoll");
  joints_to_be_clamped_to_joint_limits.push_back("rightWristPitch");

  state_sync_nasa app(lcm, cl_cfg);
  app.joints_to_be_clamped_to_joint_limits_ =
      joints_to_be_clamped_to_joint_limits;
  app.clamping_tolerance_in_degrees_ = 2.5;  // 2.5deg is OK, above will fail

  while (0 == lcm->handle())
    ;
  return 0;
}
