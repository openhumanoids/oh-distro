#ifndef __StateEstimator_h
#define __StateEstimator_h

#include <vector>
#include <iostream>
#include <string>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include "ThreadLoop.h"
#include "QueueTypes.h"

#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/drc_lcmtypes.hpp"
//#include <model-client/model-client.hpp>

#include <inertial-odometry/Odometry.hpp>
#include <leg-odometry/TwoLegOdometry.h>
#include <leg-odometry/sharedUtilities.hpp>

#include "StateEstimatorUtilities.h"
#include "JointFilters.h"

#include "SharedTypes.h"


namespace StateEstimate
{

class StateEstimator : public ThreadLoop
{
public:


  StateEstimator(
    const StateEstimate::command_switches* _switches,
    boost::shared_ptr<lcm::LCM> lcmHandle,
    AtlasStateQueue& atlasStateQueue,
    IMUQueue& imuQueue,
    PoseQueue& bdiPoseQueue,
    PoseQueue& viconQueue,
    NavQueue& matlabTruthQueue,
    INSUpdateQueue& INSUpdateQueue);
  
  StateEstimator(
      boost::shared_ptr<lcm::LCM> lcmHandle,
      messageQueues& msgQueue );

  ~StateEstimator();

protected:

  void run();

  boost::shared_ptr<lcm::LCM> mLCM;

  AtlasStateQueue& mAtlasStateQueue;
  IMUQueue& mIMUQueue;
  PoseQueue& mBDIPoseQueue;
  PoseQueue& mViconQueue;
  NavQueue& mMatlabTruthQueue;
  INSUpdateQueue& mINSUpdateQueue;
  
  // This class is not up and running yet
  messageQueues mMSGQueues;
  
  JointFilters mJointFilters;
  //  vector<float> mJointVelocities;
  
  RobotModel* robot;

  
private:
  const StateEstimate::command_switches* _mSwitches;
  std::string ERSMsgSuffix;

  drc::robot_state_t mERSMsg;
  drc::robot_state_t testing;
  drc::ins_update_request_t mDFRequestMsg;
	
  // We maintain own InerOdoEst state, to avoid potential future multi-threaded issues
  InertialOdometry::DynamicState InerOdoEst;
  InertialOdometry::Odometry inert_odo;
  unsigned long long previous_imu_utime;
  unsigned long long prevImuPacketCount;

  BotParam* _botparam;
  BotFrames* _botframes;
  
  Eigen::Isometry3d IMU_to_body;
  
  RateChange fusion_rate;
  Eigen::VectorXd fusion_rate_dummy;
  
  // LegOdometry Object
  TwoLegs::TwoLegOdometry *_leg_odo;
  TwoLegs::FK_Data fk_data;
  
  int firstpass;
  double Ts_imu; // Auto-detect the sample rate of the IMU
  int receivedIMUPackets;

  // ======== SERVICE ROUTINES ===========
  void IMUServiceRoutine(const drc::atlas_raw_imu_t &imu, bool publishERSflag, boost::shared_ptr<lcm::LCM> lcm);
};

} // end namespace

#endif // __StateEstimator_h
