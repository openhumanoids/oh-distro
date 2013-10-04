#ifndef __StateEstimator_h
#define __StateEstimator_h

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include "ThreadLoop.h"
#include "QueueTypes.h"

#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/drc_lcmtypes.hpp"

#include <inertial-odometry/Odometry.hpp>

#include "StateEstimatorUtilities.h"
#include "JointFilters.h"

namespace StateEstimate
{

class StateEstimator : public ThreadLoop
{
public:


  StateEstimator(
    boost::shared_ptr<lcm::LCM> lcmHandle,
    AtlasStateQueue& atlasStateQueue,
    IMUQueue& imuQueue,
    PoseQueue& bdiPoseQueue,
    PoseQueue& viconQueue );
  
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
  
  // This class is not up and running yet
  messageQueues mMSGQueues;
  
  
  JointFilters mJointFilters;
  //  vector<float> mJointVelocities;
  
  drc::robot_state_t mERSMsg;
  drc::robot_state_t testing;
  drc::ins_update_request_t mDFRequestMsg;
  
  
private:
  InertialOdometry::Odometry inert_odo;
  unsigned long long previous_imu_utime;
	
  BotParam* _botparam;
  BotFrames* _botframes;
  
  Eigen::Isometry3d IMU_to_body;
  
  RateChange fusion_rate;
  Eigen::VectorXd fusion_rate_dummy;
  

  
};

} // end namespace

#endif // __StateEstimator_h
