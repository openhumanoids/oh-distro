#include "Imu.hpp"

#include <iostream>
#include <multisense-lib/MultiSenseChannel.hh>
#include <lcmtypes/multisense/imu_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <cmath>

struct Imu::Imp {

  // unit conversion factors
  static constexpr double kPi = std::acos(-1);
  static constexpr double kAccelFactor = 9.81;    // from g to m/s^2
  static constexpr double kGyroFactor = kPi/180;  // from deg/sec to rad/sec 
  static constexpr double kMagFactor = 1.0;       // no conversion

  static void callbackShim(const crl::multisense::imu::Header& iHeader,
                           void* iUserData) {
    reinterpret_cast<Imp*>(iUserData)->dataCallback(iHeader);
  }

  Imu* mImu;

  Imp(Imu* iImu) {
    mImu = iImu;
  }

  void dataCallback(const crl::multisense::imu::Header& iHeader) {
  
    // publish each sample (no interpolation, just sample-and-hold)
    for (auto samp : iHeader.samples) {

      // populate message
      multisense::imu_t msg;
      msg.utime = int64_t(samp.timeSeconds)*1000000 + samp.timeMicroSeconds;
      switch (samp.type) {
      case crl::multisense::imu::Sample::Type_Accelerometer:
        msg.accel[0] = samp.x*kAccelFactor;
        msg.accel[1] = samp.y*kAccelFactor;
        msg.accel[2] = samp.z*kAccelFactor;
        break;
      case crl::multisense::imu::Sample::Type_Gyroscope:
        msg.gyro[0] = samp.x*kGyroFactor;
        msg.gyro[1] = samp.y*kGyroFactor;
        msg.gyro[2] = samp.z*kGyroFactor;
        break;
      case crl::multisense::imu::Sample::Type_Magnetometer:
        msg.mag[0] = samp.x*kMagFactor;
        msg.mag[1] = samp.y*kMagFactor;
        msg.mag[2] = samp.z*kMagFactor;
        break;
      default: break;
      }

      mImu->mPublishLcm->publish(mImu->mOutputChannel, &msg);
    }
  }
};

Imu::
Imu(crl::multisense::Channel* iChannel) : Sensor(iChannel) {
  mImp.reset(new Imp(this));
}

Imu::
~Imu() {
  stop();
}

bool Imu::
start() {
  auto status = mChannel->startStreams(crl::multisense::Source_Imu);
  if (crl::multisense::Status_Ok != status) {
    std::cerr << "failed to start imu stream (" <<
      mChannel->statusString(status) << ")" << std::endl;
    return false;
  }
  mChannel->addIsolatedCallback(&Imp::callbackShim, mImp.get());
  std::cout << "started imu" << std::endl;
  return true;
}

bool Imu::
stop() {
  mChannel->removeIsolatedCallback(&Imp::callbackShim);
  auto status = mChannel->stopStreams(crl::multisense::Source_Imu);
  if (crl::multisense::Status_Ok != status) {
    std::cerr << "failed to stop imu stream (" <<
      mChannel->statusString(status) << ")" << std::endl;
    return false;
  }
  return true;
}
