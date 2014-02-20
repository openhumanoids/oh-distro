#include "IMUFilter.h"

//-----------------------------------------------------------------------------
StateEstimate::IMUFilter::IMUFilter(const std::string &ERSChannel) : ERSMsgChannelName(ERSChannel)
{
  unsigned long long fusion_period;
  fusion_period = 9000-500;
  fusion_rate.setDesiredPeriod_us(0,fusion_period);
  fusion_rate.setSize(1);
  fusion_rate_dummy.resize(1);
  fusion_rate_dummy << 0;
  std::cout << "StateEstimate::IMUFilter::IMUFilter -- Setting data fusion trigger period to " << fusion_period << std::endl;

  uninitialized = true;
  initindex = 0;

}

//-----------------------------------------------------------------------------
StateEstimate::IMUFilter::~IMUFilter()
{

}

//-----------------------------------------------------------------------------
void StateEstimate::IMUFilter::handleIMUPackets(const std::vector<drc::atlas_raw_imu_t>& imuPackets, const bot_core::pose_t &atlasPose)
{
  // note, this runs on the LCM comm thread, so be quick!
  // Only update INS and publish existing ERS message
    //std::cout << "StateEstimate::IMUFilter::handleIMUPackets -- sees " << imuPackets.size() << " new IMU messages in the imu packet vector" << std::endl;

	// Leave initialization to the Kalman Filter -- This is legacy functionality.
	uninitialized = false;
    //uninitialized = false; -- TEMP TESTING
	_inert_odo->enterCritical();
	for (int k=0;k<imuPackets.size();k++) {
		imu_data.uts = imuPackets[k].utime;
		// We convert a delta angle into a rotation rate, and will then use this as a constant rotation rate between received messages
		// We know the KVH will sample every 1 ms.
		imu_data.dang_b = Eigen::Vector3d(imuPackets[k].delta_rotation[0], imuPackets[k].delta_rotation[1], imuPackets[k].delta_rotation[2]);
		imu_data.a_b_measured = Eigen::Vector3d(imuPackets[k].linear_acceleration[0],imuPackets[k].linear_acceleration[1],imuPackets[k].linear_acceleration[2]);
		imu_data.use_dang = true;

		if (!uninitialized) {
			*_InerOdoState = _inert_odo->PropagatePrediction(imu_data);
		} else {
		  //_inert_odo->sensedImuToBodyTransform(imu_data); // This is a bit messy -- improve abstraction
	      initacceldata.push_back(imu_data.a_b_measured);
		  initindex++;
		  if (initindex>=100) {
			_inert_odo->setInitPitchRoll(initacceldata);
			uninitialized = false;
			std::cout << "StateEstimate::IMUFilter::handleIMUPackets -- initindex imu packets at " << initindex << ". Initialization complete." << std::endl;
		  }
		}
	}
	_inert_odo->exitCritical();


	if (!uninitialized) {
		bot_core::pose_t tmp_msg;
		tmp_msg = stampInertialPoseMsgs(*_InerOdoState, _inert_odo->getIMU2Body(), *_ERSMsg, mPoseBodyMsg, _VelArrowDrawTrans, _inert_odo->getAlignmentQuaternion());
		mLCM->publish(ERSMsgChannelName, _ERSMsg);
		//stampInertialPoseBodyMsg(*_InerOdoState, _inert_odo->getIMU2Body(), mPoseBodyMsg, _VelArrowDrawTrans);
		mLCM->publish("POSE_BODY", &mPoseBodyMsg);
		//mLCM->publish("POSE_BODY_ALT", &tmp_msg );

		// EKF measurement update rate was set to 90Hz
		if (fusion_rate.genericRateChange(imu_data.uts,fusion_rate_dummy,fusion_rate_dummy)) {

			stampInertialPoseUpdateRequestMsg(*_InerOdoState, *_DFRequestMsg);

			Eigen::Vector3d refMeasurement, refVelocity;
			int updateType;

			//refMeasurement = _inert_odo->getIMU2Body().linear().transpose() * (*_filteredLegVel);
			updateType = *_legKinStateClassification;
			refVelocity = (*_filteredLegVel);

			Eigen::Isometry3d legKin;
			// NOt happy with this pitch and roll quaternion yet
			legKin = _legOdo->getKinematicsTransform(Eigen::Quaterniond::Identity());
			// We have leg kinematics heading angle encoded in legKin

			//std::cout << "StateEstimate::IMUFilter::handleIMUPackets -- LegStateClassification: " << updateType << std::endl;
			//stampEKFReferenceMeasurementUpdateRequest(Eigen::Vector3d::Zero(), drc::ins_update_request_t::VELOCITY_LOCAL, *_DFRequestMsg);
			stampEKFReferenceMeasurementUpdateRequest(refVelocity, Eigen::Quaterniond(legKin.linear()), updateType, *_DFRequestMsg);
			mLCM->publish("SE_MATLAB_DATAFUSION_REQ", _DFRequestMsg);
			//std::cout << "YAW NOW IS: " << q2e_new(_InerOdoState->lQb)[2] << std::endl;
		}

	}

  //VarNotUsed(imuPackets);
  //VarNotUsed(lcmHandle);
}

void StateEstimate::IMUFilter::setupEstimatorSharedMemory(StateEstimate::StateEstimator &estimator) {
  setInertialOdometry( estimator.getInertialOdometry() );
  setERSMsg( estimator.getERSMsg() );
  setDataFusionReqMsg( estimator.getDataFusionReqMsg() );
  setInerOdoStateContainerPtr( estimator.getInerOdoPtr() );
  setFilteredLegOdoVel( estimator.getFilteredLegOdoVel() );
  setLegStateClassification(estimator.getLegStateClassificationPtr() );
  setLegOdoPtr(estimator.getLegOdoPtr() );
  setVelArrowTransform(estimator.getVelArrowDrawTransform() );
  //setAlignTransform(estimator.getAlignTransform());
}

void StateEstimate::IMUFilter::setInertialOdometry(InertialOdometry::Odometry* _inertialOdoPtr) {
  _inert_odo = _inertialOdoPtr;
}

void StateEstimate::IMUFilter::setERSMsg(drc::robot_state_t* _msg) {
  _ERSMsg = _msg;
}

void StateEstimate::IMUFilter::setDataFusionReqMsg(drc::ins_update_request_t* _msg) {
  _DFRequestMsg = _msg;
}

void StateEstimate::IMUFilter::setLCMPtr(boost::shared_ptr<lcm::LCM> lcmHandle) {
	mLCM = lcmHandle;
}

void StateEstimate::IMUFilter::setInerOdoStateContainerPtr(InertialOdometry::DynamicState* _stateptr) {
  _InerOdoState = _stateptr;
}

void StateEstimate::IMUFilter::setFilteredLegOdoVel(Eigen::Vector3d* _legodovel) {
  _filteredLegVel = _legodovel;
}

void StateEstimate::IMUFilter::setLegStateClassification(int* ptr) {
  _legKinStateClassification = ptr;
}

void StateEstimate::IMUFilter::setLegOdoPtr(leg_odometry* _lo) {
  _legOdo = _lo;
}

void StateEstimate::IMUFilter::setVelArrowTransform(Eigen::Isometry3d* _ptr) {
  _VelArrowDrawTrans = _ptr;
}

//void StateEstimate::IMUFilter::setAlignTransform(Eigen::Isometry3d* _ptr) {
//  _align = _ptr;
//}
