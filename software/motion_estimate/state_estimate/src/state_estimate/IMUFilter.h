#ifndef __IMUFilter_h
#define __IMUFilter_h

#include <vector>

#include "LCMSubscriber.h"
#include "QueueTypes.h"

#include "lcmtypes/drc_lcmtypes.hpp"
#include <bot_frames/bot_frames.h>
#include <inertial-odometry/Odometry.hpp>

#include "StateEstimatorUtilities.h"


namespace StateEstimate
{

class IMUFilter
{
public:

  IMUFilter();
  ~IMUFilter();

  void handleIMUPackets(const std::vector<drc::atlas_raw_imu_t>& imuPackets, const bot_core::pose_t &atlasPose);
  void setInertialOdometry(InertialOdometry::Odometry* _inertialOdoPtr);
  void setERSMsg(drc::robot_state_t* _msg);
  void setDataFusionReqMsg(drc::ins_update_request_t* _msg);
  void setLCMPtr(boost::shared_ptr<lcm::LCM> lcmHandle);
  void setInerOdoStateContainerPtr(InertialOdometry::DynamicState* _stateptr);
  void setFilteredLegOdoVel(Eigen::Vector3d* _legodovel);

private:
  boost::shared_ptr<lcm::LCM> mLCM;

  InertialOdometry::Odometry* _inert_odo;
  InertialOdometry::IMU_dataframe imu_data;
  //InertialOdometry::DynamicState lastInerOdoState;
  InertialOdometry::DynamicState* _InerOdoState; // Move state to StateEstimator process too.
  drc::robot_state_t* _ERSMsg;
  drc::ins_update_request_t* _DFRequestMsg;
  bot_core::pose_t mPoseBodyMsg;

  Eigen::Vector3d* _filteredLegVel;

  RateChange fusion_rate;
  Eigen::VectorXd fusion_rate_dummy; // Unused variable -- added to class private definition for speed.

  std::vector<Eigen::Vector3d> initacceldata;
  bool uninitialized;
  int initindex;
};


} // end namespace

#endif // __IMUFilter_h
