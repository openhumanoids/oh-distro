#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>
#include <vector>
#include <fstream>

#include "state_sync.hpp"
#include <ConciseArgs>
#include <sys/time.h>



using namespace std;
#define DO_TIMING_PROFILE FALSE

/////////////////////////////////////

void onParamChangeSync(BotParam* old_botparam, BotParam* new_botparam,
                     int64_t utime, void* user) {  
  state_sync& sync = *((state_sync*)user);
  sync.setBotParam(new_botparam);
  sync.setEncodersFromParam();
}

state_sync::state_sync(boost::shared_ptr<lcm::LCM> &lcm_, 
                       boost::shared_ptr<CommandLineConfig> &cl_cfg_):
                       lcm_(lcm_), cl_cfg_(cl_cfg_){

  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));     

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 1); // 1 means keep updated, 0 would ignore updates
  bot_param_add_update_subscriber(botparam_,
                                  onParamChangeSync, this);
   
  /// Subscribe to required signals
  //lcm::Subscription* sub0 = lcm_->subscribe("ATLAS_STATE",&state_sync::atlasHandler,this); // depricated
  lcm::Subscription* sub0 = lcm_->subscribe("CORE_ROBOT_STATE",&state_sync::coreRobotHandler,this);
  lcm::Subscription* sub1 = lcm_->subscribe("FORCE_TORQUE",&state_sync::forceTorqueHandler,this);
  force_torque_init_ = false;
  ///////////////////////////////////////////////////////////////
  lcm::Subscription* sub2 = lcm_->subscribe("ATLAS_STATE_EXTRA",&state_sync::atlasExtraHandler,this);
  lcm::Subscription* sub4 = lcm_->subscribe("ENABLE_ENCODERS",&state_sync::enableEncoderHandler,this);  
  lcm::Subscription* sub5 = lcm_->subscribe("POSE_BDI",&state_sync::poseBDIHandler,this); // Always provided by the Atlas Driver:
  lcm::Subscription* sub6 = lcm_->subscribe("POSE_BODY",&state_sync::poseMITHandler,this);  // Always provided the state estimator:
  lcm::Subscription* sub7 = lcm_->subscribe("MULTISENSE_STATE",&state_sync::multisenseHandler,this);
  
  bool use_short_queue = true;
  if (use_short_queue){
    sub0->setQueueCapacity(1);
    sub1->setQueueCapacity(1);
    sub2->setQueueCapacity(1); 
    sub4->setQueueCapacity(1); 
    sub5->setQueueCapacity(1); 
    sub6->setQueueCapacity(1); 
    sub7->setQueueCapacity(1);
  }

  setPoseToZero(pose_BDI_);
  setPoseToZero(pose_MIT_);
 
  
  /// 3. Using Pots or Encoders:
  cl_cfg_->use_encoder_joint_sensors = bot_param_get_boolean_or_fail(botparam_, "control.encoder_offsets.active" );
  std::cout << "use_encoder_joint_sensors: " << cl_cfg_->use_encoder_joint_sensors << "\n";

  // explicitly identify encoder joints
  encoder_joint_indices_ = {
    Atlas::JOINT_R_ARM_SHZ,
    Atlas::JOINT_R_ARM_SHX,
    Atlas::JOINT_R_ARM_ELY,
    Atlas::JOINT_R_ARM_ELX,
    Atlas::JOINT_L_ARM_SHZ,
    Atlas::JOINT_L_ARM_SHX,
    Atlas::JOINT_L_ARM_ELY,
    Atlas::JOINT_L_ARM_ELX,

    // TODO: may not need these
    Atlas::JOINT_R_ARM_UWY,
    Atlas::JOINT_R_ARM_MWX,
    Atlas::JOINT_R_ARM_LWY,
    Atlas::JOINT_L_ARM_UWY,
    Atlas::JOINT_L_ARM_MWX,
    Atlas::JOINT_L_ARM_LWY,
  };

  // encoder offsets if encoders are used
  encoder_joint_offsets_.assign(Atlas::NUM_JOINTS,0.0);
  // Encoder now read from main cfg file and updates received via param server
  setEncodersFromParam();

  //maximum encoder angle before wrapping.  if q > max_angle, use q - 2*pi
  // if q < min_angle, use q + 2*pi
  max_encoder_wrap_angle_.assign(Atlas::NUM_JOINTS,100000000);
  max_encoder_wrap_angle_[Atlas::JOINT_R_ARM_UWY] = 4; // robot software v1.9
  max_encoder_wrap_angle_[Atlas::JOINT_R_ARM_SHZ] = 0; // robot software v1.9
  max_encoder_wrap_angle_[Atlas::JOINT_L_ARM_ELY] = 3; // robot software v1.9

  use_encoder_.assign(Atlas::NUM_JOINTS,false);
  enableEncoders(true); // disable for now


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

  string joint_filter_type = bot_param_get_str_or_fail(botparam_, "control.filtering.joints.type" );
  if (joint_filter_type == "kalman"){
    cl_cfg_->use_joint_kalman_filter = true;
  }else if(joint_filter_type == "backlash"){
    cl_cfg_->use_joint_backlash_filter = true;
  }// else none
  
  
  if (cl_cfg_->use_joint_kalman_filter || cl_cfg_->use_joint_backlash_filter){
    // Shared params:
    double process_noise_pos = bot_param_get_double_or_fail(botparam_, "control.filtering.joints.process_noise_pos" );
    double process_noise_vel = bot_param_get_double_or_fail(botparam_, "control.filtering.joints.process_noise_vel" );
    double observation_noise = bot_param_get_double_or_fail(botparam_, "control.filtering.joints.observation_noise" );

    int n_filters = bot_param_get_array_len (botparam_, "control.filtering.joints.index");
    int filter_idx[n_filters];
    bot_param_get_int_array_or_fail(botparam_, "control.filtering.joints.index", &filter_idx[0], n_filters);

    if (cl_cfg_->use_joint_kalman_filter){
      for (size_t i=0;i < n_filters; i++){
        EstimateTools::SimpleKalmanFilter* a_kf = new EstimateTools::SimpleKalmanFilter (process_noise_pos, process_noise_vel, observation_noise); // uses Eigen2d
        filter_idx_.push_back(filter_idx[i]);
        joint_kf_.push_back(a_kf) ;
      }
      std::cout << "Created " << joint_kf_.size() << " Kalman Filters with noise "<< process_noise_pos << ", " << process_noise_vel << " | " << observation_noise << "\n";

    }else if(cl_cfg_->use_joint_backlash_filter){
      double backlash_alpha = bot_param_get_double_or_fail(botparam_, "control.filtering.joints.backlash_alpha" );
      double backlash_crossing_time_max = bot_param_get_double_or_fail(botparam_, "control.filtering.joints.backlash_crossing_time_max" );

      for (size_t i=0;i < n_filters; i++){
        EstimateTools::BacklashFilter* a_kf = new EstimateTools::BacklashFilter (process_noise_pos, process_noise_vel, observation_noise); // uses Eigen2d
        a_kf->setAlpha( backlash_alpha );
        a_kf->setCrossingTimeMax( backlash_crossing_time_max );
        filter_idx_.push_back(filter_idx[i]);
        joint_backlashfilter_.push_back(a_kf) ;
      }
      std::cout << "Created " << joint_backlashfilter_.size() << " Backlash Filters with noise "<< process_noise_pos << ", " << process_noise_vel << " | " << observation_noise << "\n";
      std::cout << "Backlash alpha: " << backlash_alpha << " crossing_time_max: " << backlash_crossing_time_max << "\n";
    }

  }
  
  /// 5. Floating Base Filtering
  string rotation_rate_filter_type = bot_param_get_str_or_fail(botparam_, "control.filtering.rotation_rate.type" );
  if (rotation_rate_filter_type == "alpha" ){// alpha filter on rotation rates
    cl_cfg_->use_rotation_rate_alpha_filter = true;
    double alpha_rotation_rate = bot_param_get_double_or_fail(botparam_, "control.filtering.rotation_rate.alpha" );
    rotation_rate_alpha_filter_ = new EstimateTools::AlphaFilter (alpha_rotation_rate);
    std::cout << "Using Rotation Rate Alpha filter with " << alpha_rotation_rate << "\n";
  }
  
  /// 6. neck joint filtering
  double neck_alpha = 0.97;
  neck_alpha_filter_ = new EstimateTools::AlphaFilter (neck_alpha);
  
  utime_prev_ = 0;
}

void state_sync::setPoseToZero(PoseT &pose){
  pose.utime = 0; // use this to signify un-initalised
  pose.pos << 0,0, 0.95;// for ease of use//Eigen::Vector3d::Identity();
  pose.vel << 0,0,0;
  pose.orientation << 1.,0.,0.,0.;
  pose.rotation_rate << 0,0,0;
  pose.accel << 0,0,0;
}

void state_sync::setEncodersFromParam() {
  
  std::string str = "State Sync: refreshing offsets (from param)";
  std::cout << str << std::endl;
  // display system status message in viewer
  bot_core::system_status_t stat_msg;
  stat_msg.utime = 0;
  stat_msg.system = stat_msg.MOTION_ESTIMATION;
  stat_msg.importance = stat_msg.VERY_IMPORTANT;
  stat_msg.frequency = stat_msg.LOW_FREQUENCY;
  stat_msg.value = str;
  lcm_->publish(("SYSTEM_STATUS"), &stat_msg);  
  
  int n_indices = bot_param_get_array_len (botparam_, "control.encoder_offsets.index");  
  int n_offsets = bot_param_get_array_len (botparam_, "control.encoder_offsets.value");  
  std::cout << n_indices << " indices and " << n_offsets << " offsets\n";
  if (n_indices != n_offsets){
    std::cout << "n_indices is now n_offsets, not updating\n";
    return;
  }
    
  double offsets_in[n_indices];
  int indices_in[n_offsets];
  bot_param_get_int_array_or_fail(botparam_, "control.encoder_offsets.index", &indices_in[0], n_indices);  
  bot_param_get_double_array_or_fail(botparam_, "control.encoder_offsets.value", &offsets_in[0], n_offsets);  
  std::vector<double> indices(indices_in, indices_in + n_indices);
  std::vector<double> offsets(offsets_in, offsets_in + n_offsets);
  
  extra_offsettable_joint_indices_.clear();
  for (size_t i=0; i < indices.size() ; i++){
    // encoder_joint_offsets_[jindex] = offset;
    encoder_joint_offsets_[ indices[i] ] = offsets[i];
    std::cout << i << ": " << indices[i] << " " << offsets[i] << "\n";

    // if this is not an encoder joint that can be disabled,
    // add it to the list of offsettable joints
    if (std::find(encoder_joint_indices_.begin(), encoder_joint_indices_.end(), indices[i]) == encoder_joint_indices_.end()) {
      extra_offsettable_joint_indices_.push_back(indices[i]);
    }
  }
  std::cout << "Finished updating encoder offsets (from param)\n";  
}

void state_sync::enableEncoderHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::utime_t* msg) {
  enableEncoders(msg->utime > 0); // sneakily use utime as a flag
}


void state_sync::enableEncoders(bool enable) {
  // display system status message in viewer
  bot_core::system_status_t stat_msg;
  stat_msg.utime = 0;
  stat_msg.system = stat_msg.MOTION_ESTIMATION;
  stat_msg.importance = stat_msg.VERY_IMPORTANT;
  stat_msg.frequency = stat_msg.LOW_FREQUENCY;
  std::string str;
  if (enable) {
    str = "State Sync: enabling encoders.";
  }
  else {
    str = "State Sync: disabling encoders.";
  }
  stat_msg.value = str;
  lcm_->publish(("SYSTEM_STATUS"), &stat_msg);   

  use_encoder_[Atlas::JOINT_R_ARM_SHZ] = enable;
  use_encoder_[Atlas::JOINT_R_ARM_SHX] = enable;
  use_encoder_[Atlas::JOINT_R_ARM_ELY] = enable;
  use_encoder_[Atlas::JOINT_R_ARM_ELX] = enable;
  /* don't need to actually set these
  use_encoder_[Atlas::JOINT_R_ARM_UWY] = false; // always false for electric forearm
  use_encoder_[Atlas::JOINT_R_ARM_MWX] = false;
  use_encoder_[Atlas::JOINT_R_ARM_LWY] = false;
  */

  use_encoder_[Atlas::JOINT_L_ARM_SHZ] = enable;
  use_encoder_[Atlas::JOINT_L_ARM_SHX] = enable;
  use_encoder_[Atlas::JOINT_L_ARM_ELY] = enable;
  use_encoder_[Atlas::JOINT_L_ARM_ELX] = enable;
  /* don't need to actually set these
  use_encoder_[Atlas::JOINT_L_ARM_UWY] = false; // always false for electric forearm
  use_encoder_[Atlas::JOINT_L_ARM_MWX] = false;
  use_encoder_[Atlas::JOINT_L_ARM_LWY] = false;
  */

}

void state_sync::multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg){
  head_joints_.name = msg->joint_name;
  head_joints_.position = msg->joint_position;
  head_joints_.velocity = msg->joint_velocity;
  head_joints_.effort = msg->joint_effort;
  
  if (cl_cfg_->standalone_head){
    bot_core::six_axis_force_torque_array_t force_torque_msg;
    publishRobotState(msg->utime, force_torque_msg);
  }
  
}

void state_sync::leftHandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg){
  left_hand_joints_.name = msg->joint_name;
  left_hand_joints_.position = msg->joint_position;
  left_hand_joints_.velocity = msg->joint_velocity;
  left_hand_joints_.effort = msg->joint_effort;
  
  if (cl_cfg_->standalone_hand){ // assumes only one hand is actively publishing state
    bot_core::six_axis_force_torque_array_t force_torque_msg;
    publishRobotState(msg->utime, force_torque_msg);
  }  
}

void state_sync::rightHandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg){
  right_hand_joints_.name = msg->joint_name;
  right_hand_joints_.position = msg->joint_position;
  right_hand_joints_.velocity = msg->joint_velocity;
  right_hand_joints_.effort = msg->joint_effort;
  
  if (cl_cfg_->standalone_hand){ // assumes only one hand is actively publishing state
    bot_core::six_axis_force_torque_array_t force_torque_msg;
    publishRobotState(msg->utime, force_torque_msg);
  }    
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void state_sync::filterJoints(int64_t utime, std::vector<float> &joint_position, std::vector<float> &joint_velocity){
  //int64_t tic = _timestamp_now();
  double t = (double) utime*1E-6;
  
  
  for (size_t i=0; i <  filter_idx_.size(); i++){
    double x_filtered;
    double x_dot_filtered;
    if ( cl_cfg_->use_joint_kalman_filter ){
      joint_kf_[i]->processSample(t,  joint_position[filter_idx_[i]] , joint_velocity[filter_idx_[i]] , x_filtered, x_dot_filtered);
    }else if( cl_cfg_->use_joint_backlash_filter ){
      joint_backlashfilter_[i]->processSample(t,  joint_position[filter_idx_[i]] , joint_velocity[filter_idx_[i]] , x_filtered, x_dot_filtered);
    }
    joint_position[ filter_idx_[i] ] = x_filtered;
    joint_velocity[ filter_idx_[i] ] = x_dot_filtered;
  }
  
  // filter neck
  Eigen::VectorXd neck_state = Eigen::VectorXd::Zero(2);
  neck_state(0) = joint_position[Atlas::JOINT_NECK_AY];
  neck_state(1) = joint_velocity[Atlas::JOINT_NECK_AY];
  Eigen::VectorXd filtered_neck_state = Eigen::VectorXd::Zero(2);
  
  neck_alpha_filter_->processSample(neck_state, filtered_neck_state);

  joint_position[Atlas::JOINT_NECK_AY] = filtered_neck_state(0);
  joint_velocity[Atlas::JOINT_NECK_AY] = filtered_neck_state(1);
  
  
  //int64_t toc = _timestamp_now();
  //double dtime = (toc-tic)*1E-6;
  //std::cout << dtime << " end\n";   
}


void state_sync::forceTorqueHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::six_axis_force_torque_array_t* msg){
  force_torque_ = *msg;
  force_torque_init_ = true; 
}


void state_sync::coreRobotHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg){
  if (!force_torque_init_){
    std::cout << "FORCE_TORQUE not received yet, not publishing EST_ROBOT_STATE =========================\n";
    return;    
  }  

  std::vector <float> mod_positions;
  mod_positions = msg->joint_position;
  core_robot_joints_.position = mod_positions;

  core_robot_joints_.name = msg->joint_name;  // added recently
  core_robot_joints_.velocity = msg->joint_velocity;
  core_robot_joints_.effort = msg->joint_effort;
  
  
  // Overwrite the actuator joint positions and velocities with the after-transmission 
  // sensor values for the ARMS ONLY (first exposed in v2.7.0 of BDI's API)
  // NB: this assumes that they are provided at the same rate as ATLAS_STATE
  if (cl_cfg_->use_encoder_joint_sensors ){
    if (core_robot_joints_.position.size() == core_robot_joints_out_.position.size()   ){
      if (core_robot_joints_.velocity.size() == core_robot_joints_out_.velocity.size()   ){
        for (int i=0; i < core_robot_joints_out_.position.size() ; i++ ) { 

          if (use_encoder_[i]) {
            core_robot_joints_.position[i] = core_robot_joints_out_.position[i];
            if (core_robot_joints_.position[i] > max_encoder_wrap_angle_[i])
              core_robot_joints_.position[i] -= 2*M_PI;
            core_robot_joints_.position[i] += encoder_joint_offsets_[i];

            // check for wonky encoder initialization :(
            while (core_robot_joints_.position[i] - mod_positions[i] > 0.5)
              core_robot_joints_.position[i] -= 2*M_PI/3;
            while (core_robot_joints_.position[i] - mod_positions[i] < -0.5)
              core_robot_joints_.position[i] += 2*M_PI/3;

            /*
            if (abs(core_robot_joints_.position[i] - mod_positions[i]) > 0.11 && (msg->utime - utime_prev_ > 5000000)) {
              utime_prev_ = msg->utime;

              // display system status message in viewer
              bot_core::system_status_t stat_msg;
              stat_msg.utime = msg->utime;
              stat_msg.system = stat_msg.MOTION_ESTIMATION;
              stat_msg.importance = stat_msg.VERY_IMPORTANT;
              stat_msg.frequency = stat_msg.LOW_FREQUENCY;

              std::stringstream message;
              message << "State Sync: Joint '" << ATLAS_JOINT_NAMES[i] << "' encoder might need calibration :)";
              stat_msg.value = message.str();

              lcm_->publish(("SYSTEM_STATUS"), &stat_msg); 
              std::cout << message.str() << std::endl; 
            }

            */
            core_robot_joints_.velocity[i] = core_robot_joints_out_.velocity[i];
          }
        }
      }
    }
  }

  // apply cfg offsets to special joints
  for (auto idx : extra_offsettable_joint_indices_) {
    core_robot_joints_.position[idx] += encoder_joint_offsets_[idx];
    if (core_robot_joints_.position[idx] > max_encoder_wrap_angle_[idx]) {
      core_robot_joints_.position[idx] -= 2*M_PI;
    }
  }
  
  if (cl_cfg_->use_joint_kalman_filter || cl_cfg_->use_joint_backlash_filter ){//  core_robot_joints_ filtering here
    filterJoints(msg->utime, core_robot_joints_.position, core_robot_joints_.velocity);
  }

  if (cl_cfg_->use_torque_adjustment){
    torque_adjustment_->processSample(core_robot_joints_.position, core_robot_joints_.effort );
  }

  // TODO: check forque_
  publishRobotState(msg->utime, force_torque_);
}

void state_sync::atlasExtraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  atlas::state_extra_t* msg){
  //std::cout << "got atlasExtraHandler\n";
  core_robot_joints_out_.position = msg->joint_position_out;
  core_robot_joints_out_.velocity = msg->joint_velocity_out;
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

Eigen::Isometry3d getPoseAsIsometry3d(PoseT pose){
  Eigen::Isometry3d pose_iso;
  pose_iso.setIdentity();
  pose_iso.translation()  << pose.pos[0], pose.pos[1] , pose.pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(pose.orientation[0], pose.orientation[1], 
                                               pose.orientation[2], pose.orientation[3]);
  pose_iso.rotate(quat);
  return pose_iso;
}


void state_sync::poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  pose_BDI_.utime = msg->utime;
  pose_BDI_.pos = Eigen::Vector3d( msg->pos[0],  msg->pos[1],  msg->pos[2] );
  pose_BDI_.vel = Eigen::Vector3d( msg->vel[0],  msg->vel[1],  msg->vel[2] );
  pose_BDI_.orientation = Eigen::Vector4d( msg->orientation[0],  msg->orientation[1],  msg->orientation[2],  msg->orientation[3] );
  pose_BDI_.rotation_rate = Eigen::Vector3d( msg->rotation_rate[0],  msg->rotation_rate[1],  msg->rotation_rate[2] );
  pose_BDI_.accel = Eigen::Vector3d( msg->accel[0],  msg->accel[1],  msg->accel[2] );  
}

void state_sync::poseMITHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  pose_MIT_.utime = msg->utime;
  pose_MIT_.pos = Eigen::Vector3d( msg->pos[0],  msg->pos[1],  msg->pos[2] );
  pose_MIT_.vel = Eigen::Vector3d( msg->vel[0],  msg->vel[1],  msg->vel[2] );
  pose_MIT_.orientation = Eigen::Vector4d( msg->orientation[0],  msg->orientation[1],  msg->orientation[2],  msg->orientation[3] );
  pose_MIT_.rotation_rate = Eigen::Vector3d( msg->rotation_rate[0],  msg->rotation_rate[1],  msg->rotation_rate[2] );
  pose_MIT_.accel = Eigen::Vector3d( msg->accel[0],  msg->accel[1],  msg->accel[2] );  

  // If State sync has received POSE_BDI and POSE_BODY, we must be running our own estimator
  // So there will be a difference between these, so publish this for things like walking footstep transformations
  // TODO: rate limit this to something like 10Hz
  // TODO: this might need to be published only when pose_BDI_.utime and pose_MIT_.utime are very similar
  if ( pose_BDI_.utime > 0 ){
    Eigen::Isometry3d localmit_to_bodymit = getPoseAsIsometry3d(pose_MIT_);
    Eigen::Isometry3d localmit_to_bodybdi = getPoseAsIsometry3d(pose_BDI_);
// localmit_to_body.inverse() * localmit_to_body_bdi;
    // Eigen::Isometry3d localmit_to_localbdi = localmit_to_bodymit.inverse() * localmit_to_bodymit.inverse() * localmit_to_bodybdi;
    Eigen::Isometry3d localmit_to_localbdi = localmit_to_bodybdi * localmit_to_bodymit.inverse();

    bot_core::rigid_transform_t localmit_to_localbdi_msg = getIsometry3dAsBotRigidTransform( localmit_to_localbdi, pose_MIT_.utime );
    lcm_->publish("LOCAL_TO_LOCAL_BDI", &localmit_to_localbdi_msg);    
  }

}



// Returns false if the pose is old or hasn't appeared yet
bool state_sync::insertPoseInRobotState(bot_core::robot_state_t& msg, PoseT pose){
  // TODO: add comparison of Atlas State utime and Pose's utime
  //if (pose.utime ==0){
  //  std::cout << "haven't received pelvis pose, refusing to publish ERS\n";
  //  return false;
  //}
  
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

  
  ///////////// Rotation Rate Alpha Filter ///////////////////////////////
  Eigen::VectorXd rr_prefiltered(3), rr_filtered(3);
  rr_prefiltered << pose.rotation_rate[0], pose.rotation_rate[1], pose.rotation_rate[2] ;
  if ( cl_cfg_->use_rotation_rate_alpha_filter){
    rotation_rate_alpha_filter_->processSample(rr_prefiltered, rr_filtered    );
  }else{
    rr_filtered = rr_prefiltered; 
  }
  ////////////////////////////////////////////////////////
  
  Eigen::Vector3d rot_vel_local = R*Eigen::Vector3d ( rr_filtered[0], rr_filtered[1], rr_filtered[2] );
  msg.twist.angular_velocity.x = rot_vel_local[0];
  msg.twist.angular_velocity.y = rot_vel_local[1];
  msg.twist.angular_velocity.z = rot_vel_local[2];
  
  return true;  
}

bool insertPoseInBotState(bot_core::pose_t& msg, PoseT pose){
  // TODO: add comparison of Atlas State utime and Pose's utime
  //if (pose.utime ==0){
  //  std::cout << "haven't received pelvis pose, refusing to populated pose_t\n";
  //  return false;
  //}
  
  msg.utime = pose.utime;
  msg.pos[0] = pose.pos[0];
  msg.pos[1] = pose.pos[1];
  msg.pos[2] = pose.pos[2];
  msg.orientation[0] = pose.orientation[0];
  msg.orientation[1] = pose.orientation[1];
  msg.orientation[2] = pose.orientation[2];
  msg.orientation[3] = pose.orientation[3];

  msg.vel[0] = pose.vel[0];
  msg.vel[1] = pose.vel[1];
  msg.vel[2] = pose.vel[2];
  
  msg.rotation_rate[0] = pose.rotation_rate[0];
  msg.rotation_rate[1] = pose.rotation_rate[1];
  msg.rotation_rate[2] = pose.rotation_rate[2];
  
  msg.accel[0] = pose.accel[0];
  msg.accel[1] = pose.accel[1];
  msg.accel[2] = pose.accel[2];
  
  return true;  
}


void state_sync::publishRobotState(int64_t utime_in,  const  bot_core::six_axis_force_torque_array_t& force_torque_msg){
  
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
  appendJoints(robot_state_msg, head_joints_);
  
  appendJoints(robot_state_msg, left_hand_joints_);
  appendJoints(robot_state_msg, right_hand_joints_);

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
  
  // std::cout << "sending " << robot_state_msg.num_joints << " joints\n";

  if (cl_cfg_->simulation_mode){ // to be deprecated..
    lcm_->publish("TRUE_ROBOT_STATE", &robot_state_msg);    
  }

  if (cl_cfg_->bdi_motion_estimate){
    if ( insertPoseInRobotState(robot_state_msg, pose_BDI_) ){
      bot_core::pose_t pose_body;
      insertPoseInBotState(pose_body, pose_BDI_);
      if (cl_cfg_->publish_pose_body) lcm_->publish("POSE_BODY", &pose_body);
      lcm_->publish( cl_cfg_->output_channel  , &robot_state_msg);
    }
  }else if(cl_cfg_->standalone_head || cl_cfg_->standalone_hand ){
    if ( insertPoseInRobotState(robot_state_msg, pose_MIT_) ){
      lcm_->publish( cl_cfg_->output_channel , &robot_state_msg);
    }
  }else{ // typical motion estimation
    if ( insertPoseInRobotState(robot_state_msg, pose_MIT_) ){
      lcm_->publish( cl_cfg_->output_channel, &robot_state_msg);
    }
  }
}

void state_sync::appendJoints(bot_core::robot_state_t& msg_out, Joints joints){
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
  opt.add(cl_cfg->standalone_head, "l", "standalone_head","Standalone Head");
  opt.add(cl_cfg->standalone_hand, "f", "standalone_hand","Standalone Hand");
  opt.add(cl_cfg->bdi_motion_estimate, "b", "bdi","Use POSE_BDI to make EST_ROBOT_STATE");
  opt.add(cl_cfg->simulation_mode, "s", "simulation","Simulation mode - output TRUE RS");
  opt.add(cl_cfg->output_channel, "o", "output_channel","Output Channel for robot state msg");
  opt.add(cl_cfg->publish_pose_body, "p", "publish_pose_body","Publish POSE_BODY when in BDI mode");
  opt.add(cl_cfg->atlas_version, "a", "atlas_version", "Atlas version to use");
  opt.parse();
  
  std::cout << "standalone_head: " << cl_cfg->standalone_head << "\n";
  std::cout << "publish_pose_body: " << cl_cfg->publish_pose_body << "\n";

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  state_sync app(lcm, cl_cfg);
  while(0 == lcm->handle());
  return 0;
}
