#ifndef MULTISENSE_ROS_IMU_H
#define MULTISENSE_ROS_IMU_H

#include <MultiSenseChannel.hh>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/multisense/imu_t.hpp>

namespace multisense_ros {

class Imu {
public:
  Imu(crl::multisense::Channel* driver);
  ~Imu();

  void dataCallback(const crl::multisense::imu::Header& header);

private:
  void start();
  void stop();

  crl::multisense::Channel* driver_;

  lcm::LCM lcm_publish_ ;
  multisense::imu_t latest_msg_;
};

}

#endif
