#ifndef __IMUBatchProcessor_h
#define __IMUBatchProcessor_h

#include "LCMSubscriber.h"
#include "QueueTypes.h"


namespace StateEstimate
{

class IMUBatchProcessor
{
private:
	unsigned long last_utime;
	
public:

  IMUBatchProcessor()
  {
    last_utime = 0;
  }

  void handleIMUBatchMessage(const drc::atlas_raw_imu_batch_t* msg, std::vector<drc::atlas_raw_imu_t>& imuPackets)
  {
    // fill the imuPackets vector with only the new imu packets
    // ...

	std::cout << "handleIMUBatchMessage -- is happening " << (msg->raw_imu[0].utime > last_utime) << std::endl;
	  
    // for now, just copy the first 3 packets...
    // for (int i = 0; i < 3; ++i)
	for (int i = 0; msg->raw_imu[i].utime > last_utime; i++) 
    {
      last_utime = msg->raw_imu[i].utime;
      imuPackets.push_back(msg->raw_imu[i]);
    }
  }


};


} // end namespace

#endif // __IMUBatchProcessor_h
