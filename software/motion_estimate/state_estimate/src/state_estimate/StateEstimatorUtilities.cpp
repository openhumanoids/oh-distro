#include "StateEstimatorUtilities.h"

bool StateEstimate::convertBDIPose_ERS(const bot_core::pose_t* msg, drc::robot_state_t& ERS_msg) {
  PoseT pose_container;
  extractBDIPose(msg, pose_container);
  return insertPoseBDI(pose_container, ERS_msg);
}

void StateEstimate::extractBDIPose(const bot_core::pose_t* msg, PoseT &pose_BDI_) {
  pose_BDI_.utime = msg->utime;
  pose_BDI_.pos = Eigen::Vector3d( msg->pos[0],  msg->pos[1],  msg->pos[2] );
  pose_BDI_.vel = Eigen::Vector3d( msg->vel[0],  msg->vel[1],  msg->vel[2] );
  pose_BDI_.orientation = Eigen::Vector4d( msg->orientation[0],  msg->orientation[1],  msg->orientation[2],  msg->orientation[3] );
  pose_BDI_.rotation_rate = Eigen::Vector3d( msg->rotation_rate[0],  msg->rotation_rate[1],  msg->rotation_rate[2] );
  pose_BDI_.accel = Eigen::Vector3d( msg->accel[0],  msg->accel[1],  msg->accel[2] );  
}


bool StateEstimate::insertPoseBDI(const PoseT &pose_BDI_, drc::robot_state_t& msg) {
  
  // This won't work, because another messages can potentially insert information in a different order-- 
  // so we assume a large update time delta will a be a reason for returning false (50ms)
  if (pose_BDI_.utime + 50000 < msg.utime) {
	  std::cout << "StateEstimate::insertPoseBDI -- trying to overwrite ERS pose message with stale data" << std::endl;
	  return false;
  }
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



bool StateEstimate::insertAtlasState_ERS(const drc::atlas_state_t &atlasState, drc::robot_state_t &mERSMsg){

	Joints jointsContainer;
	
	// This is called form state_sync
	insertAtlasJoints(&atlasState, jointsContainer);
	appendJoints(mERSMsg, jointsContainer);
	
	return false;
}




void StateEstimate::appendJoints(drc::robot_state_t& msg_out, const StateEstimate::Joints &joints){
  for (size_t i = 0; i < joints.position.size(); i++)  {
    msg_out.joint_name.push_back( joints.name[i] );
    msg_out.joint_position.push_back( joints.position[i] );
    msg_out.joint_velocity.push_back( joints.velocity[i] );
    msg_out.joint_effort.push_back( joints.effort[i] );
  }
}


void StateEstimate::insertAtlasJoints(const drc::atlas_state_t* msg, StateEstimate::Joints &jointContainer) {

  jointContainer.name = msg->joint_name;
  jointContainer.position = msg->joint_position;
  jointContainer.velocity = msg->joint_velocity;
  jointContainer.effort = msg->joint_effort;

  return;
}



void StateEstimate::handle_inertial_data_temp_name(const double dt, const drc::atlas_raw_imu_t &imu, const bot_core::pose_t &bdiPose, const Eigen::Isometry3d &IMU_to_body, InertialOdometry::Odometry &inert_odo, drc::robot_state_t& _ERSmsg, drc::ins_update_request_t& _DFRequest) {
  
  // We want to take body fram einerial data an put it in the imu_data structure
  // For now we are going to use the orientation quaternion from BDI
  // We will do r best to subtract a constant gravity -- this happens in the inertia odometry process
  // THis is double integrated and we return the velocity and position estimates
  // We assume that this objet will be updated by some magic LCM message from somewhere else
	
  InertialOdometry::DynamicState InerOdoEst;
  InertialOdometry::IMU_dataframe imu_data;
	
  // Using the BDI quaternion estimate for now
  Eigen::Quaterniond q(bdiPose.orientation[0],bdiPose.orientation[1],bdiPose.orientation[2],bdiPose.orientation[3]);
  
  std::cout << "StateEstimate::handle_inertial_data_temp_name -- q = " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
  
  imu_data.uts = imu.utime;
  
  
  
  // TODO -- The translation of this lever arm offset hsa not yet been compensated! This must be corrected.
  imu_data.gyr_b = IMU_to_body.linear() * Eigen::Vector3d(imu.delta_rotation[0], imu.delta_rotation[1], imu.delta_rotation[2]);
  imu_data.acc_b = IMU_to_body.linear() * Eigen::Vector3d(imu.linear_acceleration[0],imu.linear_acceleration[1],imu.linear_acceleration[2]);
  
  std::cout << "StateEstimate::handle_inertial_data_temp_name -- acc before pelvis alignment " << Eigen::Vector3d(imu.linear_acceleration[0],imu.linear_acceleration[1],imu.linear_acceleration[2]).transpose() << std::endl;
  
  InerOdoEst = inert_odo.PropagatePrediction(&imu_data, q);
  
  // For now we just patch this data directly through to the ERS message
  // TODO -- check that we want to keep doing with fusion from a MATLAB process. 
  // Think it should be good, but this is a breadcrum for the future to make sure that we do this correctly
  
  _DFRequest.pose.rotation.w = InerOdoEst.q.w();
  _DFRequest.pose.rotation.x = InerOdoEst.q.x();
  _DFRequest.pose.rotation.y = InerOdoEst.q.y();
  _DFRequest.pose.rotation.z = InerOdoEst.q.z();
  
  _DFRequest.local_linear_acceleration.x = InerOdoEst.f_l(0);
  _DFRequest.local_linear_acceleration.y = InerOdoEst.f_l(1);
  _DFRequest.local_linear_acceleration.z = InerOdoEst.f_l(2);
  
  // remember that this will have to publish a LCM message 
  _ERSmsg.pose.translation.x = InerOdoEst.P(0);
  _ERSmsg.pose.translation.y = InerOdoEst.P(1);
  _ERSmsg.pose.translation.z = InerOdoEst.P(2);
  
  _DFRequest.pose.translation.x = InerOdoEst.P(0);
  _DFRequest.pose.translation.y = InerOdoEst.P(1);
  _DFRequest.pose.translation.z = InerOdoEst.P(2);
  
  _ERSmsg.twist.linear_velocity.x = InerOdoEst.V(0);
  _ERSmsg.twist.linear_velocity.y = InerOdoEst.V(1);
  _ERSmsg.twist.linear_velocity.z = InerOdoEst.V(2);
  
  _DFRequest.twist.linear_velocity.x = InerOdoEst.V(0);
  _DFRequest.twist.linear_velocity.y = InerOdoEst.V(1);
  _DFRequest.twist.linear_velocity.z = InerOdoEst.V(2);
  
  // Rotate the body measured rotation rates with the current best known quaternion -- for now we use the BDIPOose orientation estimate
  
  _ERSmsg.pose.rotation.w = bdiPose.orientation[0];
  _ERSmsg.pose.rotation.x = bdiPose.orientation[1];
  _ERSmsg.pose.rotation.y = bdiPose.orientation[2];
  _ERSmsg.pose.rotation.z = bdiPose.orientation[3];
  
  Eigen::Vector3d rates_b(imu.delta_rotation[0]/dt, imu.delta_rotation[1]/dt, imu.delta_rotation[3]/dt);
  
  // TODO -- The bit below should be moved into the inertial odometry class, and then be available in the system by default
  Eigen::Vector3d rates_l;
  rates_l = inert_odo.C_bw() * rates_b;
  
  // Insert local frame rates estimates in the ERS message
  _ERSmsg.twist.angular_velocity.x = rates_l(0);
  _ERSmsg.twist.angular_velocity.x = rates_l(1);
  _ERSmsg.twist.angular_velocity.x = rates_l(2);
  
  return;
}


void StateEstimate::doLegOdometry(TwoLegs::FK_Data &_fk_data, const drc::atlas_state_t &atlasState, const bot_core::pose_t &_bdiPose, TwoLegs::TwoLegOdometry &_leg_odo, int firstpass) {
	
  // Keep joint positions in local memory -- prepare data structure for use with FK
  std::map<std::string, double> jointpos_in;
  for (uint i=0; i< (uint) atlasState.num_joints; i++) {
	jointpos_in.insert(make_pair(atlasState.joint_name[i], atlasState.joint_position[i]));
  }
  
  Eigen::Isometry3d current_pelvis;
  Eigen::VectorXd pelvis_velocity(3);

  Eigen::Isometry3d left;
  left.setIdentity();
  Eigen::Isometry3d right;
  right.setIdentity();
  
  // TODO -- Delete head_to_body transform requirement here. This is legacy from VRC -- 
  Eigen::Isometry3d body_to_head;
  body_to_head.setIdentity();
  
  
  _fk_data.utime = atlasState.utime;
  _fk_data.jointpos_in = jointpos_in;
  
  TwoLegs::getFKTransforms(_fk_data, left, right, body_to_head);// FK, translations in body frame with no rotation (I)
  
  // TODO -- Initialization before the VRC..
  if (firstpass>0)
  {
    Eigen::Isometry3d init_state;
    init_state.setIdentity();
    _leg_odo.ResetWithLeftFootStates(left,right,init_state);
  }
}

//  // This must be broken into separate position and velocity states
//  legchangeflag = _leg_odo->UpdateStates(_msg->utime, left, right, left_force, right_force); //footstep propagation happens in here -- we assume that body to world quaternion is magically updated by torso_imu
//  if (legchangeflag) {
//	  persistlegchangeflag = true; // this is to bridge the rate change gap
//  }
//
//  current_pelvis = _leg_odo->getPelvisState();	
//}


//int StateEstimate::getIMUBodyAlignment(const unsigned long &utime, Eigen::Isometry3d &IMU_to_body, boost::shared_ptr<lcm::LCM> &lcm_) : lcm_(lcm_) {
//
//	// TODO -- these may be repeated at the leg-odometry code, to solve for the leg kinematics.
//	// for this however we are only concerned
//	BotParam* _botparam;
//	BotFrames* _botframes;
//	
//	_botparam = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
//	_botframes= bot_frames_get_global(lcm_->getUnderlyingLCM(), _botparam);
//	
//	IMU_to_body.setIdentity();
//	
//    int status;
//    double matx[16];
//    status = bot_frames_get_trans_mat_4x4_with_utime( _botframes, "body",  "imu", utime, matx);
//    for (int i = 0; i < 4; ++i) {
//      for (int j = 0; j < 4; ++j) {
//        IMU_to_body(i,j) = matx[i*4+j];
//      }
//    }
//    
//    return status;
//}


