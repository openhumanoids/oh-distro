#ifndef __IMUBatchProcessor_h
#define __IMUBatchProcessor_h

#include "LCMSubscriber.h"
#include "QueueTypes.h"


namespace StateEstimate
{

class IMUBatchProcessor
{
public:

  IMUBatchProcessor()
  {

  }

  void handleIMUBatchMessage(const drc::atlas_raw_imu_batch_t* msg, std::vector<drc::atlas_raw_imu_t>& imuPackets)
  {
    // fill the imuPackets vector with only the new imu packets
    // ...

    // for now, just copy the first 3 packets...
    for (int i = 0; i < 3; ++i)
    {
      imuPackets.push_back(msg->raw_imu[i]);
    }
  }


};


} // end namespace

#endif // __IMUBatchProcessor_h
