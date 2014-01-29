#include "IMUFilter.h"

//-----------------------------------------------------------------------------
StateEstimate::IMUFilter::IMUFilter()
{

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
  // std::cout << "StateEstimate::IMUFilter::handleIMUPackets -- sees " << imuPackets.size() << " new IMU messages in the imu packet vector" << std::endl;

  _inert_odo->enterCritical();
  for (int k=0;k<imuPackets.size();k++) {
	imu_data.uts = imuPackets[k].utime;
	// We convert a delta angle into a rotation rate, and will then use this as a constant rotation rate between received messages
	// We know the KVH will sample every 1 ms.
	imu_data.dang_b = Eigen::Vector3d(imuPackets[k].delta_rotation[0], imuPackets[k].delta_rotation[1], imuPackets[k].delta_rotation[2]);
	imu_data.a_b_measured = Eigen::Vector3d(imuPackets[k].linear_acceleration[0],imuPackets[k].linear_acceleration[1],imuPackets[k].linear_acceleration[2]);
	imu_data.use_dang = true;

	lastInerOdoState = _inert_odo->PropagatePrediction(imu_data);
  }
  _inert_odo->exitCritical();

  stampInertialPoseERSMsg(lastInerOdoState, *_ERSMsg);
  mLCM->publish("EST_ROBOT_STATE", _ERSMsg);

  // EKF measurement update rate set to 20ms here
  if ((lastInerOdoState.uts % 20000) == 0) {
	stampInertialPoseUpdateRequestMsg(lastInerOdoState, *_DFRequestMsg);
	stampEKFReferenceMeasurementUpdateRequest(Eigen::Vector3d::Zero(), drc::ins_update_request_t::VELOCITY_LOCAL, *_DFRequestMsg);
	mLCM->publish("SE_MATLAB_DATAFUSION_REQ", _DFRequestMsg);
  }


  //VarNotUsed(imuPackets);
  //VarNotUsed(lcmHandle);
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



