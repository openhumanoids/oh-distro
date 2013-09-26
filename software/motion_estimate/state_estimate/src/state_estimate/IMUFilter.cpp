#include "IMUFilter.h"

//-----------------------------------------------------------------------------
StateEstimate::IMUFilter::IMUFilter()
{

}

//-----------------------------------------------------------------------------
StateEstimate::IMUFilter::~IMUFilter()
{

}

//-----------------------------------------------------------------------------
void StateEstimate::IMUFilter::handleIMUPackets(const std::vector<drc::atlas_raw_imu_t>& imuPackets, boost::shared_ptr<lcm::LCM> lcmHandle, const bot_core::pose_t &atlasPose)
{
  // do something with imu packets and publish lcm message
  // note, this runs on the LCM comm thread, so be quick!

  // TODO -- dehann, insert data passing to INS object here- hereafter the entire ERS message will be published from the calling function
  
  
	
  
  VarNotUsed(imuPackets);
  VarNotUsed(lcmHandle);
}
