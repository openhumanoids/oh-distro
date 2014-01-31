#ifndef __IMUFilter_h
#define __IMUFilter_h

#include <vector>

#include "LCMSubscriber.h"
#include "QueueTypes.h"

#include "lcmtypes/drc_lcmtypes.hpp"
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

private:
  boost::shared_ptr<lcm::LCM> mLCM;

  InertialOdometry::Odometry* _inert_odo;
  InertialOdometry::IMU_dataframe imu_data;
  InertialOdometry::DynamicState lastInerOdoState;
  drc::robot_state_t* _ERSMsg;
  drc::ins_update_request_t* _DFRequestMsg;

  RateChange fusion_rate;
  Eigen::VectorXd fusion_rate_dummy; // Unused variable -- added to class private definition for speed.

  std::vector<Eigen::Vector3d> initacceldata;
  bool uninitialized;
  int initindex;
};


} // end namespace

#endif // __IMUFilter_h
