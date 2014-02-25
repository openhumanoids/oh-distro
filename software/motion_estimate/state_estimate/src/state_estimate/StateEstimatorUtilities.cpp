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

void StateEstimate::stampInertialPoseMsgs(const InertialOdometry::DynamicState &InerOdoEst,
											const Eigen::Isometry3d &IMU_to_body,
											drc::robot_state_t& ERSmsg,
											bot_core::pose_t &_msg,
											Eigen::Isometry3d *_mArrowTransform,
											const Eigen::Quaterniond &alignq_out) {
  Eigen::Quaterniond outq, aliasout;
  Eigen::Vector3d P_w, V_leverarm;

  // Compute output transforms
  outq = (qprod(C2q(IMU_to_body.linear().transpose()), InerOdoEst.lQb.conjugate()));
  aliasout = qprod(outq, alignq_out);
  outq = aliasout;

  P_w = IMU_to_body.linear() * InerOdoEst.P + IMU_to_body.translation();

  V_leverarm = IMU_to_body.linear() * InerOdoEst.V + qrot(outq,(InerOdoEst.w_b).cross( IMU_to_body.linear().transpose() * IMU_to_body.translation()));

  // populate fields
  ERSmsg.utime = InerOdoEst.uts;

  ERSmsg.pose.rotation.w = outq.w();
  ERSmsg.pose.rotation.x = outq.x();
  ERSmsg.pose.rotation.y = outq.y();
  ERSmsg.pose.rotation.z = outq.z();

  //Compensate for radial components due to translation offset

  copyDrcVec3D(V_leverarm, ERSmsg.twist.linear_velocity);
  copyDrcVec3D(P_w, ERSmsg.pose.translation);
  copyDrcVec3D(IMU_to_body.linear() * InerOdoEst.w_l, ERSmsg.twist.angular_velocity);

  // Also do the POSE_BODY message
  _msg.pos[0] = P_w(0);
  _msg.pos[1] = P_w(1);
  _msg.pos[2] = P_w(2);

  _msg.orientation[0] = outq.w();
  _msg.orientation[1] = outq.x();
  _msg.orientation[2] = outq.y();
  _msg.orientation[3] = outq.z();

  _mArrowTransform->linear() = IMU_to_body.linear() * q2C(outq);
  _mArrowTransform->translation() = P_w;

  return;
}



//void StateEstimate::doLegOdometry(TwoLegs::FK_Data &_fk_data, const drc::atlas_state_t &atlasState, const bot_core::pose_t &_bdiPose, TwoLegs::TwoLegOdometry &_leg_odo, int firstpass, RobotModel* _robot) {
//
//  // Keep joint positions in local memory -- prepare data structure for use with FK
//  std::map<std::string, double> jointpos_in;
//  for (uint i=0; i< (uint) atlasState.num_joints; i++) {
//	//jointpos_in.insert(make_pair(atlasState.joint_name[i], atlasState.joint_position[i]));
//
//	// Changing name types to AtlasControlTypes definition
//	jointpos_in.insert(make_pair(_robot->joint_names_[i], atlasState.joint_position[i]));
//	//std::cout << "StateEstimate::doLegOdometry -- inserting joint " << robot.joint_names_[i] << ", " << atlasState.joint_position[i] << std::endl;
//  }
//
//  Eigen::Isometry3d current_pelvis;
//  Eigen::VectorXd pelvis_velocity(3);
//
//  Eigen::Isometry3d left;
//  left.setIdentity();
//  Eigen::Isometry3d right;
//  right.setIdentity();
//
//  // TODO -- Delete head_to_body transform requirement here. This is legacy from VRC --
//  Eigen::Isometry3d body_to_head;
//  body_to_head.setIdentity();
//
//
//  _fk_data.utime = atlasState.utime;
//  _fk_data.jointpos_in = jointpos_in;
//
//  TwoLegs::getFKTransforms(_fk_data, left, right, body_to_head);// FK, translations in body frame with no rotation (I)
//
//  // TODO -- Initialization before the VRC..
//  if (firstpass>0)
//  {
//    Eigen::Isometry3d init_state;
//    init_state.setIdentity();
//    _leg_odo.ResetWithLeftFootStates(left,right,init_state);
//  }
//  _leg_odo.UpdateStates(atlasState.utime, left, right, atlasState.force_torque.l_foot_force_z, atlasState.force_torque.r_foot_force_z); //footstep propagation happens in here -- we assume that body to world quaternion is magically updated by torso_imu
//
//
//}


//void StateEstimate::packDFUpdateRequestMsg(InertialOdometry::Odometry &inert_odo, TwoLegs::TwoLegOdometry &_leg_odo, drc::ins_update_request_t &msg) {
void StateEstimate::stampInertialPoseUpdateRequestMsg(const InertialOdometry::DynamicState &InerOdoEst, drc::ins_update_request_t &msg) {
  
  msg.utime = InerOdoEst.uts;
  
  msg.pose.rotation.w = InerOdoEst.lQb.w();
  msg.pose.rotation.x = InerOdoEst.lQb.x();
  msg.pose.rotation.y = InerOdoEst.lQb.y();
  msg.pose.rotation.z = InerOdoEst.lQb.z();
  
  //std::cout << "StateEstimate::stampInertialPoseUpdateRequestMsg -- INS estimated heading now is " << q2e_new(InerOdoEst.lQb)[2] << std::endl;

  copyDrcVec3D(InerOdoEst.a_l, msg.local_linear_acceleration);
  copyDrcVec3D(InerOdoEst.f_l, msg.local_linear_force);
  copyDrcVec3D(InerOdoEst.V, msg.twist.linear_velocity);
  copyDrcVec3D(InerOdoEst.w_l, msg.twist.angular_velocity);
  copyDrcVec3D(InerOdoEst.P, msg.pose.translation);
  copyDrcVec3D(InerOdoEst.bg, msg.gyroBiasEst);
  copyDrcVec3D(InerOdoEst.ba, msg.accBiasEst);
  copyDrcVec3D(InerOdoEst.a_b, msg.predicted_a_b);
  copyDrcVec3D(InerOdoEst.w_b, msg.predicted_w_b);

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

void StateEstimate::stampEKFReferenceMeasurementUpdateRequest(const Eigen::Vector3d &_ref, const Eigen::Quaterniond &refLegKinQ, const int type, drc::ins_update_request_t &msg) {

	// Set defaults
	copyDrcVec3D(Eigen::Vector3d::Zero(), msg.referencePos_local);
	copyDrcVec3D(Eigen::Vector3d::Zero(), msg.referenceVel_local);
	copyDrcVec3D(Eigen::Vector3d::Zero(), msg.referenceVel_body);

	msg.referenceQ_local.w = refLegKinQ.w();
	msg.referenceQ_local.x = refLegKinQ.x();
	msg.referenceQ_local.y = refLegKinQ.y();
	msg.referenceQ_local.z = refLegKinQ.z();

	Eigen::Vector3d E;
	E = q2e_new(refLegKinQ);
	msg.reference_wanderAzimHeading = E(2);

	//std::cout << "StateEstimate::stampEKFReferenceMeasurementUpdateRequest -- ref heading is " << E[2] << std::endl;

	switch (type) {
	case drc::ins_update_request_t::VEL_HEADING_LOCAL:
		//copyDrcVec3D(_ref, msg.referencePos_local);
		copyDrcVec3D(_ref, msg.referenceVel_local);
//		reference_wanderAzimHeading = ??;
		break;
	case drc::ins_update_request_t::VELOCITY_LOCAL:
		copyDrcVec3D(_ref, msg.referenceVel_local);
		break;
	case drc::ins_update_request_t::VELOCITY_BODY:
		copyDrcVec3D(_ref, msg.referenceVel_body);
		break;
	case drc::ins_update_request_t::NO_MEASUREMENT:
		//std::cout << "StateEstimate::stampEKFReferenceMeasurementUpdateRequest -- no measurement update." << std::endl;
		break;
	default:
		std::cerr << "StateEstimate::stampEKFReferenceMeasurementUpdateRequest -- requesting invalid EKF update request type." << std::endl;
		break;
	}

	msg.updateType = type;

	return;
}

void StateEstimate::copyDrcVec3D(const Eigen::Vector3d &from, drc::vector_3d_t &to) {
  to.x = from(0);
  to.y = from(1);
  to.z = from(2);
}

// Depreciated HIL test function
//void StateEstimate::stampMatlabReferencePoseUpdateRequest(const drc::nav_state_t &matlabPose, drc::ins_update_request_t &msg) {
//
//	msg.updateType = drc::ins_update_request_t::MATLAB_TRAJ_ALL;
//
//	msg.referencePos_local.x = matlabPose.pose.translation.x;
//	msg.referencePos_local.y = matlabPose.pose.translation.y;
//	msg.referencePos_local.z = matlabPose.pose.translation.z;
//
//	msg.referenceVel_local.x = matlabPose.twist.linear_velocity.x;
//	msg.referenceVel_local.y = matlabPose.twist.linear_velocity.y;
//	msg.referenceVel_local.z = matlabPose.twist.linear_velocity.z;
//
//	msg.referenceVel_body.x = 0.;
//	msg.referenceVel_body.y = 0.;
//	msg.referenceVel_body.z = 0.;
//
//	msg.referenceQ_local.w = matlabPose.pose.rotation.w;
//	msg.referenceQ_local.x = matlabPose.pose.rotation.x;
//	msg.referenceQ_local.y = matlabPose.pose.rotation.y;
//	msg.referenceQ_local.z = matlabPose.pose.rotation.z;
//}


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


void StateEstimate::detectIMUSampleTime(unsigned long long &prevImuPacketCount,
										  unsigned long long &previous_imu_utime,
										  int &receivedIMUPackets,
										  double &previous_Ts_imu,
										  const drc::atlas_raw_imu_t &imu) {

	// Auto-detect the sample rate of the IMU -- mainly for testing from different IMUs, should be 1kHz for KVH on Atlas and probably 100Hz for Microstrain
	unsigned long long deltaPacket;
	double Ts_imu;
	Ts_imu = 0;
	if (prevImuPacketCount == 0) {
		prevImuPacketCount = imu.packet_count;
		previous_imu_utime = imu.utime;
		Ts_imu = 1E-2;
	} else {
		if (imu.packet_count > prevImuPacketCount) {
			deltaPacket = imu.packet_count - prevImuPacketCount;
			if (deltaPacket > 1) {
				std::cout << "StateEstimate::detectIMUSampleTime -- " << deltaPacket-1 << " missing IMU packets!" << std::endl;
			}
			else
			{
				receivedIMUPackets++;
				Ts_imu = (imu.utime - previous_imu_utime)*1.E-6/deltaPacket;
				previous_imu_utime = imu.utime;
				std::cout << "StateEstimate::detectIMUSampleTime -- deltaPacket computed as: " << deltaPacket << ", Ts_imu set to " << Ts_imu << std::endl;
			}
		} else {
			if (prevImuPacketCount != 0) {
				std::cerr << "StateEstimate::detectIMUSampleTime -- non-monotonic IMU packet count!!! assuming " << Ts_imu << " s spacing between packets." << std::endl;
			}
		}
	}
	prevImuPacketCount = imu.packet_count;
	previous_Ts_imu = Ts_imu;
}






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


