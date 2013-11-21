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



bool StateEstimate::insertAtlasState_ERS(const drc::atlas_state_t &atlasState, drc::robot_state_t &mERSMsg, RobotModel* _robot){

	Joints jointsContainer;
	
	// This is called form state_sync
	insertAtlasJoints(&atlasState, jointsContainer, _robot);
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


void StateEstimate::insertAtlasJoints(const drc::atlas_state_t* msg, StateEstimate::Joints &jointContainer, RobotModel* _robot) {

  jointContainer.name = _robot->joint_names_;
  jointContainer.position = msg->joint_position;
  jointContainer.velocity = msg->joint_velocity;
  jointContainer.effort = msg->joint_effort;

  return;
}



void StateEstimate::handle_inertial_data_temp_name(
		const double dt,
		const drc::atlas_raw_imu_t &imu,
		const bot_core::pose_t &bdiPose,
		const Eigen::Isometry3d &IMU_to_body,
		InertialOdometry::Odometry &inert_odo,
		drc::robot_state_t& _ERSmsg,
		drc::ins_update_request_t& _DFRequest) {
  
  // We want to take body frame inerial data an put it in the imu_data structure
  // For now we are going to use the orientation quaternion from BDI
  // We will do r best to subtract a constant gravity -- this happens in the inertia odometry process
  // THis is double integrated and we return the velocity and position estimates
  // We assume that this objet will be updated by some magic LCM message from somewhere else
	
  InertialOdometry::DynamicState InerOdoEst;
  InertialOdometry::IMU_dataframe imu_data;
	
  // Using the BDI quaternion estimate for now
  Eigen::Quaterniond q(bdiPose.orientation[0],bdiPose.orientation[1],bdiPose.orientation[2],bdiPose.orientation[3]);
  q.setIdentity();

  
  std::cout << "StateEstimate::handle_inertial_data_temp_name -- q = " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
  
  imu_data.uts = imu.utime;
  
  // TODO -- The translation of this lever arm offset has not yet been compensated! This must be corrected.

  // We expect to see some LCM pack loss -- this is how we choose to deal with it.
  // We convert a delta angle into a rotation rate, and will then use this as a constant rotation rate between received messages
  // We know that eh KVH will sample at every 1 ms, so that is also the rotation rate. We later assume the time for which the
  // platform is maintaining that rotation rate.
  imu_data.gyro_ = 1E3 * IMU_to_body.linear() * Eigen::Vector3d(imu.delta_rotation[0], imu.delta_rotation[1], imu.delta_rotation[2]);
  imu_data.dang_b = IMU_to_body.linear() * Eigen::Vector3d(imu.delta_rotation[0], imu.delta_rotation[1], imu.delta_rotation[2]);
  imu_data.acc_b = IMU_to_body.linear() * Eigen::Vector3d(imu.linear_acceleration[0],imu.linear_acceleration[1],imu.linear_acceleration[2]);
  
  std::cout << "StateEstimate::handle_inertial_data_temp_name -- acc before pelvis alignment " << Eigen::Vector3d(imu.linear_acceleration[0],imu.linear_acceleration[1],imu.linear_acceleration[2]).transpose() << std::endl;
  
  // Estimate our own orientation estimate
  InerOdoEst = inert_odo.PropagatePrediction(&imu_data);
  

  // This is the unit test block for the INS POSE state estimate -- used in conjunction with ground truth


  // For now we just patch this data directly through to the ERS message
  // TODO -- check that we want to keep doing with fusion from a MATLAB process. 
  // Think it should be good, but this is a bread crum for the future to make sure that we do this correctly
  
  _DFRequest.pose.rotation.w = InerOdoEst.q.w();
  _DFRequest.pose.rotation.x = InerOdoEst.q.x();
  _DFRequest.pose.rotation.y = InerOdoEst.q.y();
  _DFRequest.pose.rotation.z = InerOdoEst.q.z();

  _DFRequest.local_linear_acceleration.x = InerOdoEst.a_l(0);
  _DFRequest.local_linear_acceleration.y = InerOdoEst.a_l(1);
  _DFRequest.local_linear_acceleration.z = InerOdoEst.a_l(2);
  
  _DFRequest.local_linear_force.x = InerOdoEst.f_l(0);
  _DFRequest.local_linear_force.y = InerOdoEst.f_l(1);
  _DFRequest.local_linear_force.z = InerOdoEst.f_l(2);


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
  
  Eigen::Vector3d temp;
  temp = inert_odo.imu_compensator.getGyroBiases();

  _DFRequest.gyroBiasEst.x = temp(0);
  _DFRequest.gyroBiasEst.y = temp(1);
  _DFRequest.gyroBiasEst.z = temp(2);


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


void StateEstimate::doLegOdometry(TwoLegs::FK_Data &_fk_data, const drc::atlas_state_t &atlasState, const bot_core::pose_t &_bdiPose, TwoLegs::TwoLegOdometry &_leg_odo, int firstpass, RobotModel* _robot) {
	
  // Keep joint positions in local memory -- prepare data structure for use with FK
  std::map<std::string, double> jointpos_in;
  for (uint i=0; i< (uint) atlasState.num_joints; i++) {
	//jointpos_in.insert(make_pair(atlasState.joint_name[i], atlasState.joint_position[i]));

	// Changing name types to AtlasControlTypes definition
	jointpos_in.insert(make_pair(_robot->joint_names_[i], atlasState.joint_position[i]));
	//std::cout << "StateEstimate::doLegOdometry -- inserting joint " << robot.joint_names_[i] << ", " << atlasState.joint_position[i] << std::endl;
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
  _leg_odo.UpdateStates(atlasState.utime, left, right, atlasState.force_torque.l_foot_force_z, atlasState.force_torque.r_foot_force_z); //footstep propagation happens in here -- we assume that body to world quaternion is magically updated by torso_imu
}


//void StateEstimate::packDFUpdateRequestMsg(InertialOdometry::Odometry &inert_odo, TwoLegs::TwoLegOdometry &_leg_odo, drc::ins_update_request_t &msg) {
void StateEstimate::stampInertialPoseUpdateRequestMsg(InertialOdometry::Odometry &inert_odo, drc::ins_update_request_t &msg) {
  
  // For ease of reading we collect local copies of variables
  //std::cout << "StateEstimate::packDFUpdateRequestMsg -- leg odo translation estimate " << LegOdoPelvis.translation().transpose() << std::endl;
  
  InertialOdometry::DynamicState _insState;
  _insState = inert_odo.getDynamicState();
  
  msg.utime = _insState.uts;
  msg.pose.translation.x = _insState.P(0);
  msg.pose.translation.y = _insState.P(1);
  msg.pose.translation.z = _insState.P(2);
  
  msg.pose.rotation.w = _insState.q.w();
  msg.pose.rotation.x = _insState.q.x();
  msg.pose.rotation.y = _insState.q.y();
  msg.pose.rotation.z = _insState.q.z();
  
  msg.twist.linear_velocity.x = _insState.V(0);
  msg.twist.linear_velocity.y = _insState.V(1);
  msg.twist.linear_velocity.z = _insState.V(2);
  
  msg.twist.angular_velocity.x = _insState.w_l(0);
  msg.twist.angular_velocity.y = _insState.w_l(1);
  msg.twist.angular_velocity.z = _insState.w_l(2);
  
  return;
}

void StateEstimate::stampLegOdoPoseUpdateRequestMsg(TwoLegs::TwoLegOdometry &_leg_odo, drc::ins_update_request_t &msg) {

  // For ease of reading we collect local copies of variables
  //std::cout << "StateEstimate::packDFUpdateRequestMsg -- leg odo translation estimate " << LegOdoPelvis.translation().transpose() << std::endl;

  Eigen::Isometry3d pelvis;

  pelvis = _leg_odo.getPelvisState();

  msg.pose.translation.x = pelvis.translation().x();
  msg.pose.translation.y = pelvis.translation().y();
  msg.pose.translation.z = pelvis.translation().z();

}

void StateEstimate::stampPositionReferencePoseUpdateRequest(const Eigen::Vector3d &_refPos, drc::ins_update_request_t &msg) {
	  msg.updateType =  drc::ins_update_request_t::POSITION_LOCAL;

	  //  msg.referencePos_local.x = LegOdoPelvis.translation().x();
	  //  msg.referencePos_local.y = LegOdoPelvis.translation().y();
	  //  msg.referencePos_local.z = LegOdoPelvis.translation().z();
	  copyDrcVec3D(_refPos, msg.referencePos_local);


	  // These are unused, but initialized as a precaution
	  copyDrcVec3D(Eigen::Vector3d::Zero(), msg.referenceVel_local);

	  msg.referenceVel_body.x = 0.;
	  msg.referenceVel_body.y = 0.;
	  msg.referenceVel_body.z = 0.;

	  msg.referenceQ_local.w = 1.;
	  msg.referenceQ_local.x = 0.;
	  msg.referenceQ_local.y = 0.;
	  msg.referenceQ_local.z = 0.;

}

void StateEstimate::copyDrcVec3D(const Eigen::Vector3d &from, drc::vector_3d_t &to) {
  to.x = from(0);
  to.y = from(1);
  to.z = from(2);
}

void StateEstimate::stampMatlabReferencePoseUpdateRequest(const drc::nav_state_t &matlabPose, drc::ins_update_request_t &msg) {

	msg.updateType = drc::ins_update_request_t::MATLAB_TRAJ_ALL;

	msg.referencePos_local.x = matlabPose.pose.translation.x;
	msg.referencePos_local.y = matlabPose.pose.translation.y;
	msg.referencePos_local.z = matlabPose.pose.translation.z;

	msg.referenceVel_local.x = matlabPose.twist.linear_velocity.x;
	msg.referenceVel_local.y = matlabPose.twist.linear_velocity.y;
	msg.referenceVel_local.z = matlabPose.twist.linear_velocity.z;

	msg.referenceVel_body.x = 0.;
	msg.referenceVel_body.y = 0.;
	msg.referenceVel_body.z = 0.;

	msg.referenceQ_local.w = matlabPose.pose.rotation.w;
	msg.referenceQ_local.x = matlabPose.pose.rotation.x;
	msg.referenceQ_local.y = matlabPose.pose.rotation.y;
	msg.referenceQ_local.z = matlabPose.pose.rotation.z;
}


void StateEstimate::onMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg, RobotModel* robot) {
  // Received robot urdf string. Store it internally and get all available joints.

   robot->robot_name      = msg->robot_name;
   robot->urdf_xml_string = msg->urdf_xml_string;
  std::cout<<"Received urdf_xml_string of robot ["<<msg->robot_name <<"], storing it internally as a param"<<std::endl;

  urdf::Model robot_model;
  if (!robot_model.initString( msg->urdf_xml_string))
  {std::cerr << "ERROR: Could not generate robot model" << std::endl;}

  typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joints_mapType;
  for( joints_mapType::const_iterator it = robot_model.joints_.begin(); it!=robot_model.joints_.end(); it++)
  {
	  if(it->second->type!=6) // All joints that not of the type FIXED.
               robot->joint_names_.push_back(it->first);//Joint names are sorted in alphabetical order within the urdf::Model structure.
  }
 }//end onMessage

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


