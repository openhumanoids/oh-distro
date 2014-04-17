#include "imu.h"

#include <limits>
#include <chrono>

using namespace crl::multisense;
using namespace multisense_ros;

namespace {
  void imuCB(const imu::Header& header, void* userDataP) {
    reinterpret_cast<Imu*>(userDataP)->dataCallback(header);
  }
}


Imu::
Imu(crl::multisense::Channel* driver) : driver_(driver) {

  // stop the imu stream
  stop();

  /*
  // query imu configuration data (for debugging)
  std::vector<imu::Config> configs;
  std::vector<imu::Info> infos;
  uint32_t maxSamplesPerMessage;
  uint32_t samplesPerMessage;
  driver_->getImuInfo(maxSamplesPerMessage, infos);
  driver_->getImuConfig(samplesPerMessage, configs);
  printf("max samples %d, cur samples %d\n", maxSamplesPerMessage,
         samplesPerMessage);
  printf("infos %d configs %d\n", infos.size(), configs.size());
  for (int i = 0; i < infos.size(); ++i) {
    printf("%s %s %s\n", infos[i].name.c_str(), infos[i].device.c_str(),
           infos[i].units.c_str());
    for (int j = 0; j < infos[i].rates.size(); ++j) {
      printf("  rate %d %f %f\n", j, infos[i].rates[j].sampleRate,
             infos[i].rates[j].bandwidthCutoff);
    }
    for (int j = 0; j < infos[i].ranges.size(); ++j) {
      printf("  range %d %f %f\n", j, infos[i].ranges[j].range,
             infos[i].ranges[j].resolution);
    }
  }
  for (int i = 0; i < configs.size(); ++i) {
    printf("%s %d %d %d\n", configs[i].name.c_str(), configs[i].enabled,
           configs[i].rateTableIndex, configs[i].rangeTableIndex);
  }
  */

  // TODO: could set desired sample rate here

  // zero message
  auto& msg = latest_msg_;
  msg.accel[0] = msg.accel[1] = msg.accel[2] = 0;
  msg.gyro[0] = msg.gyro[1] = msg.gyro[2] = 0;
  msg.mag[0] = msg.mag[1] = msg.mag[2] = 0;

  // start the imu stream
  start();

  // add callback
  driver_->addIsolatedCallback(imuCB, this);
}

Imu::
~Imu() {
  stop();
}


void Imu::
start() {
  Status status = driver_->startStreams(Source_Imu);
  if (Status_Ok != status) {
    printf("Failed to start imu stream. Error code %d", status);
  }
}

void Imu::
stop() {
  Status status = driver_->stopStreams(Source_Imu);
  if (Status_Ok != status) {
    printf("Failed to stop imu stream. Error code %d", status);
  }
}

void Imu::
dataCallback(const imu::Header& header) {

  // unit conversion factors
  const double pi = 3.14159265358979323846;
  const double accel_factor = 9.81;   // convert to m/s^2 from g
  const double gyro_factor = pi/180;  // convert to rad/sec from deg/sec
  const double mag_factor = 1.0;      // no conversion

  // compute time offset between current wall clock and latest sample
  int64_t wall_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  int64_t last_sample_time;
  {
    auto samp = header.samples.back();
    last_sample_time = samp.timeSeconds*1000000 + samp.timeMicroSeconds;
  }
  int64_t time_offset = wall_time - last_sample_time;
  
  // publish each sample (no interpolation, just sample-and-hold)
  for (auto samp : header.samples) {

    // account for time offset
    int64_t raw_time = samp.timeSeconds*1000000 + samp.timeMicroSeconds;
    int64_t utime = raw_time + time_offset;

    // populate message
    multisense::imu_t& msg = latest_msg_;
    msg.utime = utime;
    switch (samp.type) {
    case imu::Sample::Type_Accelerometer:
      msg.accel[0] = samp.x*accel_factor;
      msg.accel[1] = samp.y*accel_factor;
      msg.accel[2] = samp.z*accel_factor;
      break;
    case imu::Sample::Type_Gyroscope:
      msg.gyro[0] = samp.x*gyro_factor;
      msg.gyro[1] = samp.y*gyro_factor;
      msg.gyro[2] = samp.z*gyro_factor;
      break;
    case imu::Sample::Type_Magnetometer:
      msg.mag[0] = samp.x*mag_factor;
      msg.mag[1] = samp.y*mag_factor;
      msg.mag[2] = samp.z*mag_factor;
      break;
    default: break;
    }

    lcm_publish_.publish("MULTISENSE_IMU", &msg);
  }
}
