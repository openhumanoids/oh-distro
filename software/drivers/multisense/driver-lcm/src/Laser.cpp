#include "Laser.hpp"

#include <iostream>
#include <multisense-lib/MultiSenseChannel.hh>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/multisense/state_t.hpp>
#include <lcmtypes/multisense/planar_lidar_t.hpp>
#include <lcmtypes/multisense/rigid_transform_t.hpp>
#include <cmath>

struct Laser::Imp {

  static constexpr double kPi = std::acos(-1);
  static constexpr double kRpmLimit = 50;
  static constexpr double kAngleOffset = kPi;

  static void callbackShim(const crl::multisense::lidar::Header& iHeader,
                           void* iUserData) {
    reinterpret_cast<Imp*>(iUserData)->scanCallback(iHeader);
  }


  Laser* mLaser;
  std::string mStateChannel;
  multisense::state_t mStateMessage;

  Imp(Laser* iLaser) {
    mLaser = iLaser;

    // initialize state message; only a single joint as per ros driver
    mStateMessage.joint_name.push_back("motor_joint");
    mStateMessage.joint_position.push_back(0.0);
    mStateMessage.joint_velocity.push_back(0.0);
    mStateMessage.joint_effort.push_back(0.0);
    mStateMessage.num_joints = mStateMessage.joint_name.size();
  }

  void publishJointState(const int64_t iTime, const double iAngle,
                         const double iVelocity) {
    // convert from microradians to radians
    double angle = 1e-6*iAngle;

    // publish state_t message
    auto& stateMsg = mStateMessage;
    stateMsg.utime = iTime;
    stateMsg.joint_position[0] = angle - kAngleOffset;
    stateMsg.joint_velocity[0] = iVelocity;
    mLaser->mPublishLcm->publish(mStateChannel, &stateMsg);

    // publish message for bot_frames
    multisense::rigid_transform_t preToPostFrame;
    preToPostFrame.utime = iTime;
    preToPostFrame.trans[0] = 0;
    preToPostFrame.trans[1] = 0;
    preToPostFrame.trans[2] = 0;
    preToPostFrame.quat[0] = std::cos(angle/2);
    preToPostFrame.quat[1] = 0;
    preToPostFrame.quat[2] = 0;
    preToPostFrame.quat[3] = std::sin(angle/2);
    mLaser->mPublishLcm->publish("PRE_SPINDLE_TO_POST_SPINDLE",
                                 &preToPostFrame);
  }

  void scanCallback(const crl::multisense::lidar::Header& iHeader) {
    int64_t startAbsoluteUtime =
      iHeader.timeStartSeconds*1e6 + iHeader.timeStartMicroSeconds;
    int64_t endAbsoluteUtime =
      iHeader.timeEndSeconds*1e6 + iHeader.timeEndMicroSeconds;
    double scanTime = (endAbsoluteUtime - startAbsoluteUtime)*1e-6; // in sec

    double angleDiff = 1e-6*(iHeader.spindleAngleEnd-iHeader.spindleAngleStart);
    if (angleDiff < -kPi) angleDiff += 2*kPi;
    else if (angleDiff > kPi) angleDiff -= 2*kPi;
    const double velocity = angleDiff / scanTime;

    // publish joint state for beginning and end of scan
    publishJointState(startAbsoluteUtime, iHeader.spindleAngleStart, velocity);
    publishJointState(endAbsoluteUtime, iHeader.spindleAngleEnd, velocity);

    // publish laser scan data
    const double arcRadians = 1e-6*iHeader.scanArc;
    multisense::planar_lidar_t msg;
    msg.utime = endAbsoluteUtime;  // was startAbsoluteUtime until 04-2014
    msg.ranges.resize(iHeader.pointCount);
    msg.intensities.resize(iHeader.pointCount);
    for (size_t i = 0; i < iHeader.pointCount; ++i) {
      msg.ranges[i] = iHeader.rangesP[i] / 1000.0f;  // millimeters to meters
      msg.intensities[i] = iHeader.intensitiesP[i];  // in device units
    }
    msg.nranges = msg.ranges.size();
    msg.nintensities = msg.intensities.size();
    msg.rad0 = -arcRadians / 2.0;
    msg.radstep = arcRadians / (iHeader.pointCount - 1);
    mLaser->mPublishLcm->publish(mLaser->mOutputChannel, &msg);
  }

};

Laser::
Laser(crl::multisense::Channel* iChannel) : Sensor(iChannel) {
  mImp.reset(new Imp(this));
  setSpindleSpeed(0);
}

Laser::
~Laser() {
  stop();
}

void Laser::
setStateChannel(const std::string& iChannel) {
  mImp->mStateChannel = iChannel;
}

bool Laser::
setSpindleSpeed(const float iRpm) {
  if (std::abs(iRpm) > mImp->kRpmLimit) {
    std::cout << "warning: requested rpm " << iRpm <<
      "is beyond limit " << mImp->kRpmLimit << std::endl;
    return false;
  }
  auto status = mChannel->setMotorSpeed(iRpm);
  if (crl::multisense::Status_Ok != status) {
    std::cerr << "error: cannot set spindle speed to " << iRpm <<
      "rpm (" << mChannel->statusString(status) << ")" << std::endl;
    return false;
  }
  return true;
}

bool Laser::
start() {
  auto status = mChannel->startStreams(crl::multisense::Source_Lidar_Scan);
  if (crl::multisense::Status_Ok != status) {
    std::cerr << "failed to start laser stream (" <<
      mChannel->statusString(status) << ")" << std::endl;
    return false;
  }
  mChannel->addIsolatedCallback(&Imp::callbackShim, mImp.get());
  std::cout << "started laser" << std::endl;
  return true;
}

bool Laser::
stop() {
  mChannel->removeIsolatedCallback(&Imp::callbackShim);
  auto status = mChannel->stopStreams(crl::multisense::Source_Lidar_Scan);
  if (crl::multisense::Status_Ok != status) {
    std::cerr << "failed to stop laser stream (" <<
      mChannel->statusString(status) << ")" << std::endl;
    return false;
  }
  setSpindleSpeed(0);
  return true;
}
