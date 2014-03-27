#ifndef __IMUBatchProcessor_h
#define __IMUBatchProcessor_h

#include "LCMSubscriber.h"
#include "QueueTypes.h"


namespace StateEstimate
{

class IMUBatchProcessor
{
private:
	unsigned long long last_utime;
	
public:

  IMUBatchProcessor()
  {
    last_utime = 0;
  }

  void handleIMUBatchMessage(const drc::atlas_raw_imu_batch_t* msg, std::vector<drc::atlas_raw_imu_t>& imuPackets)
  {

	// First we need to find how far back we need to go in the imu message batch
	int i;
	//for (i = 0; msg->raw_imu[i].utime >= last_utime; i++) {;}
	for (i = msg->num_packets-1; i>=0; i--)
    {
	  if (msg->raw_imu[i].utime > last_utime) {
		  //std::cout << "handleIMUBatchMessage -- new IMU message at utime " << msg->raw_imu[i].utime << std::endl;
		  last_utime = msg->raw_imu[i].utime;
		  imuPackets.push_back(msg->raw_imu[i]);
	  }
    }
  }


};


} // end namespace

#endif // __IMUBatchProcessor_h
