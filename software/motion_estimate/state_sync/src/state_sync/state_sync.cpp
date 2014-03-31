#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>
#include <vector>
#include <fstream>
#include <math.h>

#include "state_sync.hpp"
#include <ConciseArgs>
#include <sys/time.h>



using namespace std;
#define DO_TIMING_PROFILE FALSE

/////////////////////////////////////

void assignJointsStruct( Joints &joints ){
  joints.velocity.assign( joints.name.size(), 0);
  joints.position.assign( joints.name.size(), 0);
  joints.effort.assign( joints.name.size(), 0);  
}


void onParamChangeSync(BotParam* old_botparam, BotParam* new_botparam,
                     int64_t utime, void* user) {  
  state_sync& sync = *((state_sync*)user);
  sync.setBotParam(new_botparam);
  sync.setEncodersFromParam();
}

state_sync::state_sync(boost::shared_ptr<lcm::LCM> &lcm_, 
                       bool standalone_head_, bool standalone_hand_,  
                       bool bdi_motion_estimate_, bool simulation_mode_,
                       bool use_encoder_joint_sensors_, std::string output_channel_,
                       bool publish_pose_body_,
                       bool use_kalman_filtering_):
   lcm_(lcm_), 
   standalone_head_(standalone_head_), standalone_hand_(standalone_hand_),
   bdi_motion_estimate_(bdi_motion_estimate_), simulation_mode_(simulation_mode_),
   use_encoder_joint_sensors_(use_encoder_joint_sensors_), output_channel_(output_channel_),
   publish_pose_body_(publish_pose_body_), use_kalman_filtering_(use_kalman_filtering_){
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));     

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 1); // 1 means keep updated, 0 would ignore updates
  bot_param_add_update_subscriber(botparam_,
                                  onParamChangeSync, this);
   
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
  }else if(find(joint_names.begin(), joint_names.end(), "left_finger_1_joint_1" ) != joint_names.end()){
    std::cout << "Robot fitted with left Robotiq hand\n";
    lcm_->subscribe("ROBOTIQ_LEFT_STATE",&state_sync::leftHandHandler,this);  
    left_hand_joints_.name = joint_utils_.robotiq_l_joint_names;
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
  }else if(find(joint_names.begin(), joint_names.end(), "right_finger_1_joint_1" ) != joint_names.end()){
    std::cout << "Robot fitted with right Robotiq hand\n";
    lcm_->subscribe("ROBOTIQ_RIGHT_STATE",&state_sync::rightHandHandler,this);  
    right_hand_joints_.name = joint_utils_.robotiq_r_joint_names;
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
  
  lcm::Subscription* sub0 = lcm_->subscribe("ATLAS_STATE",&state_sync::atlasHandler,this);
  ///////////////////////////////////////////////////////////////
  lcm::Subscription* sub1 = lcm_->subscribe("ATLAS_STATE_EXTRA",&state_sync::atlasExtraHandler,this);  
  lcm::Subscription* sub2 = lcm_->subscribe("ATLAS_POT_OFFSETS",&state_sync::potOffsetHandler,this);  
  //lcm::Subscription* sub3 = lcm_->subscribe("REFRESH_ENCODER_OFFSETS",&state_sync::refreshEncoderCalibrationHandler,this);  
  lcm::Subscription* sub4 = lcm_->subscribe("ENABLE_ENCODERS",&state_sync::enableEncoderHandler,this);  
  lcm::Subscription* sub5 = lcm_->subscribe("POSE_BDI",&state_sync::poseBDIHandler,this); // Always provided by the Atlas Driver:
  lcm::Subscription* sub6 =lcm_->subscribe("POSE_BODY",&state_sync::poseMITHandler,this);  // Always provided the state estimator:
  
  setPoseToZero(pose_BDI_);
  setPoseToZero(pose_MIT_);
  
  bool use_short_queue = true;
  if (use_short_queue){
    sub0->setQueueCapacity(1);
    sub1->setQueueCapacity(1); 
    sub2->setQueueCapacity(1); 
    //sub3->setQueueCapacity(1); 
    sub4->setQueueCapacity(1); 
    sub5->setQueueCapacity(1); 
    sub6->setQueueCapacity(1); 
  }
 
  
  /// Pots and Encoders:
  // pot offsets if pots are used, currently uncalibrated
  pot_joint_offsets_.assign(28,0.0);
  // encoder offsets if encoders are used
  encoder_joint_offsets_.assign(28,0.0);

  // Encoder now read from main cfg file and updates received via param server
  setEncodersFromParam();
  //loadEncoderOffsetsFromFile();
  //encoder_joint_offsets_[Atlas::JOINT_NECK_AY] = 4.24;  // robot software v1.10

  //maximum encoder angle before wrapping.  if q > max_angle, use q - 2*pi
  // if q < min_angle, use q + 2*pi
  max_encoder_wrap_angle_.assign(28,100000000);
  max_encoder_wrap_angle_[Atlas::JOINT_R_ARM_UWY] = 4; // robot software v1.9
  max_encoder_wrap_angle_[Atlas::JOINT_R_ARM_USY] = 0; // robot software v1.9
  max_encoder_wrap_angle_[Atlas::JOINT_L_ARM_ELY] = 3; // robot software v1.9

  use_encoder_.assign(28,false);
  enableEncoders(true);

  utime_prev_ = 0;
  
  
  if (use_kalman_filtering_){

    double process_noise = bot_param_get_double_or_fail(botparam_, "control.filtering.process_noise" );
    double observation_noise = bot_param_get_double_or_fail(botparam_, "control.filtering.observation_noise" );
    
    int n_filters = bot_param_get_array_len (botparam_, "control.filtering.index");  
    int filter_idx[n_filters];
    bot_param_get_int_array_or_fail(botparam_, "control.filtering.index", &filter_idx[0], n_filters);  
    for (size_t i=0;i < n_filters; i++){
      EstimateTools::SimpleKalmanFilter* a_kf = new EstimateTools::SimpleKalmanFilter (process_noise, observation_noise); // uses Eigen2d
      filter_idx_.push_back(filter_idx[i]);
      joint_kf_.push_back(a_kf) ;
    }
    std::cout << "Created " << joint_kf_.size() << " Kalman Filters with noise "<< process_noise << ", " << observation_noise << "\n";
  }
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
  drc::system_status_t stat_msg;
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
  
  for (size_t i=0; i < indices.size() ; i++){
    // encoder_joint_offsets_[jindex] = offset;
    encoder_joint_offsets_[ indices[i] ] = offsets[i];
    std::cout << i << ": " << indices[i] << " " << offsets[i] << "\n";
  }
  std::cout << "Finished updating encoder offsets (from param)\n";  
}

void state_sync::enableEncoderHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg) {
  enableEncoders(msg->utime > 0); // sneakily use utime as a flag
}


void state_sync::enableEncoders(bool enable) {
  // display system status message in viewer
  drc::system_status_t stat_msg;
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

  use_encoder_[Atlas::JOINT_R_ARM_USY] = enable;
  use_encoder_[Atlas::JOINT_R_ARM_SHX] = enable;
  use_encoder_[Atlas::JOINT_R_ARM_ELY] = enable;
  use_encoder_[Atlas::JOINT_R_ARM_ELX] = enable;
  use_encoder_[Atlas::JOINT_R_ARM_UWY] = enable;
  use_encoder_[Atlas::JOINT_R_ARM_MWX] = enable;

  use_encoder_[Atlas::JOINT_L_ARM_USY] = enable;
  use_encoder_[Atlas::JOINT_L_ARM_SHX] = enable;
  use_encoder_[Atlas::JOINT_L_ARM_ELY] = enable;
  use_encoder_[Atlas::JOINT_L_ARM_ELX] = enable;
  use_encoder_[Atlas::JOINT_L_ARM_UWY] = enable;
  use_encoder_[Atlas::JOINT_L_ARM_MWX] = enable;
  
  use_encoder_[Atlas::JOINT_NECK_AY] = enable;
}

/*
void state_sync::loadEncoderOffsetsFromFile() {
  // load encoder offsets from file

  std::string str = "State Sync: refreshing offsets";
  std::cout << str << std::endl;

  // display system status message in viewer
  drc::system_status_t stat_msg;
  stat_msg.utime = 0;
  stat_msg.system = stat_msg.MOTION_ESTIMATION;
  stat_msg.importance = stat_msg.VERY_IMPORTANT;
  stat_msg.frequency = stat_msg.LOW_FREQUENCY;
  stat_msg.value = str;
  lcm_->publish(("SYSTEM_STATUS"), &stat_msg);   

  char* drcpath = getenv("DRC_BASE");
  if (drcpath==NULL) {
    std::cout << "State Sync: error reading DRC_BASE environment variable..." << std::endl;
  }
  else {
    std::ifstream file;
    std::string filename = std::string(drcpath) + "/software/config/encoder_offsets.cfg";
    file.open(filename.c_str());
    std::string value;
    double offset;
    int jindex;
    while (file.good()) {
      getline (file, value, ','); // read a string until next comma
      jindex = ::atoi(value.c_str());
      // std::cout << "joint index: " << jindex << std::endl;
      getline (file, value, ',');
      offset = ::atof(value.c_str());
      // std::cout << "offset: " <<  offset << std::endl;
      encoder_joint_offsets_[jindex] = offset;
    }
    file.close();
  }
}
*/

//void state_sync::refreshEncoderCalibrationHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg) {
//  loadEncoderOffsetsFromFile();
//}

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
    joint_kf_[i]->processSample(t,  joint_position[filter_idx_[i]] , joint_velocity[filter_idx_[i]] , x_filtered, x_dot_filtered);
    joint_position[ filter_idx_[i] ] = x_filtered;
    joint_velocity[ filter_idx_[i] ] = x_dot_filtered;
  }
  
  //int64_t toc = _timestamp_now();
  //double dtime = (toc-tic)*1E-6;
  //std::cout << dtime << " end\n";   
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

            if (abs(atlas_joints_.position[i] - mod_positions[i]) > 0.11 && (msg->utime - utime_prev_ > 5000000)) {
              utime_prev_ = msg->utime;

              // display system status message in viewer
              drc::system_status_t stat_msg;
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

            atlas_joints_.velocity[i] = atlas_joints_out_.velocity[i];
          }
        }
      }
    }
  }

  if (use_kalman_filtering_){//  atlas_joints_ filtering here
    filterJoints(msg->utime, atlas_joints_.position, atlas_joints_.velocity);
  }
  
  publishRobotState(msg->utime, msg->force_torque);
}

void state_sync::atlasExtraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_extra_t* msg){
  //std::cout << "got atlasExtraHandler\n";
  atlas_joints_out_.position = msg->joint_position_out;
  atlas_joints_out_.velocity = msg->joint_velocity_out;
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
bool insertPoseInRobotState(drc::robot_state_t& msg, PoseT pose){
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
  // convention is for EST_ROBOT_STATE to be in linear frame
  // convert here:
  Eigen::Matrix3d R = Eigen::Matrix3d( Eigen::Quaterniond( pose.orientation[0], pose.orientation[1], pose.orientation[2],pose.orientation[3] ));
  Eigen::Vector3d lin_vel_local = R*Eigen::Vector3d ( pose.vel[0], pose.vel[1], pose.vel[2]);
  Eigen::Vector3d rot_vel_local = R*Eigen::Vector3d ( pose.rotation_rate[0], pose.rotation_rate[1], pose.rotation_rate[2]);
  
  msg.twist.linear_velocity.x = lin_vel_local[0];
  msg.twist.linear_velocity.y = lin_vel_local[1];
  msg.twist.linear_velocity.z = lin_vel_local[2];
  
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
    if ( insertPoseInRobotState(robot_state_msg, pose_BDI_) ){
      bot_core::pose_t pose_body;
      insertPoseInBotState(pose_body, pose_BDI_);
      if (publish_pose_body_) lcm_->publish("POSE_BODY", &pose_body); 
      lcm_->publish( output_channel_  , &robot_state_msg); 
    }
  }else if(standalone_head_ || standalone_hand_ ){
    lcm_->publish( output_channel_ , &robot_state_msg);
  }else{ // typical motion estimation
    if ( insertPoseInRobotState(robot_state_msg, pose_MIT_) ){
      lcm_->publish( output_channel_, &robot_state_msg);    
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
  bool standalone_hand = false;
  bool bdi_motion_estimate = false;
  bool simulation_mode = false;
  bool use_encoder_joint_sensors = false;
  bool publish_pose_body = true;
  bool use_kalman_filtering = false;
  string output_channel = "EST_ROBOT_STATE";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(standalone_head, "l", "standalone_head","Standalone Head");
  opt.add(standalone_hand, "f", "standalone_hand","Standalone Hand");
  opt.add(bdi_motion_estimate, "b", "bdi","Use POSE_BDI to make EST_ROBOT_STATE");
  opt.add(simulation_mode, "s", "simulation","Simulation mode - output TRUE RS");
  opt.add(use_encoder_joint_sensors, "e", "encoder","Use the encoder joint sensors (in the arms)");
  opt.add(output_channel, "o", "output_channel","Output Channel for robot state msg");
  opt.add(publish_pose_body, "p", "publish_pose_body","Publish POSE_BODY when in BDI mode");
  opt.add(use_kalman_filtering, "k", "kalman_filter","Use Kalman filtering on joints");
  opt.parse();
  
  std::cout << "standalone_head: " << standalone_head << "\n";
  std::cout << "publish_pose_body: " << publish_pose_body << "\n";
  std::cout << "Use transmission joint sensors: " << use_encoder_joint_sensors << " (arms only)\n";
  std::cout << "Use kalman filters: " << use_kalman_filtering << "\n";

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  state_sync app(lcm, standalone_head,standalone_hand,bdi_motion_estimate, 
                 simulation_mode, use_encoder_joint_sensors,	
		 output_channel, publish_pose_body, use_kalman_filtering);
  while(0 == lcm->handle());
  return 0;
}

