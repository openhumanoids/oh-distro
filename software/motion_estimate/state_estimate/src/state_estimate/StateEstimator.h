#ifndef __StateEstimator_h
#define __StateEstimator_h

#include "ThreadLoop.h"
#include "QueueTypes.h"

#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/drc_lcmtypes.hpp"

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
  
};

} // end namespace

#endif // __StateEstimator_h
