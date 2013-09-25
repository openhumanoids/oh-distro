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
void StateEstimate::IMUFilter::handleIMUPackets(const std::vector<drc::atlas_raw_imu_t>& imuPackets, boost::shared_ptr<lcm::LCM> lcmHandle)
{
  // do something with imu packets and publish lcm message
  // note, this runs on the LCM comm thread, so be quick!

  VarNotUsed(imuPackets);
  VarNotUsed(lcmHandle);
}
