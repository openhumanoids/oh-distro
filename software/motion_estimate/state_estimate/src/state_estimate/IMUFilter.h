#ifndef __IMUFilter_h
#define __IMUFilter_h

#include "LCMSubscriber.h"
#include "QueueTypes.h"


namespace StateEstimate
{

class IMUFilter
{
public:

  IMUFilter();
  ~IMUFilter();

  void handleIMUPackets(const std::vector<drc::atlas_raw_imu_t>& imuPackets, boost::shared_ptr<lcm::LCM> lcmHandle, const bot_core::pose_t &atlasPose);

};


} // end namespace

#endif // __IMUFilter_h
