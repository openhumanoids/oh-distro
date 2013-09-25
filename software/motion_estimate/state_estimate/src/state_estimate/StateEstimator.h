#ifndef __StateEstimator_h
#define __StateEstimator_h

#include "ThreadLoop.h"
#include "QueueTypes.h"

#include <lcm/lcm-cpp.hpp>

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

  ~StateEstimator();

protected:

  void run();

  boost::shared_ptr<lcm::LCM> mLCM;

  AtlasStateQueue& mAtlasStateQueue;
  IMUQueue& mIMUQueue;
  PoseQueue& mBDIPoseQueue;
  PoseQueue& mViconQueue;
};

} // end namespace

#endif // __StateEstimator_h
