#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>

#include <ConciseArgs>
#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/Clock.hpp>
#include <bot_core/rotations.h>

#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>
#include <lcmtypes/drc/utime_t.hpp>

struct Publisher {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  int mPublishFrequency;
  std::thread mThread;
  bool mGotMessage;
  double mRoll;
  double mPitch;
  double mYaw;

  Publisher() {
    mPublishFrequency = 100;
    mRoll = mPitch = mYaw = 0;
    mLcmWrapper.reset(new drc::LcmWrapper());
    drc::Clock::instance()->setVerbose(false);
    drc::Clock::instance()->useRealTimeWhenInvalid(false);
    drc::Clock::instance()->setLcm(mLcmWrapper->get());
  }

  void start() {
    mLcmWrapper->startHandleThread(false);
    mThread = std::thread(std::ref(*this));
    mThread.join();
  }

  void fillVector(double* iVect, const double iV0, const double iV1,
                  const double iV2) {
    iVect[0] = iV0;  iVect[1] = iV1;  iVect[2] = iV2;
  }

  void fillVector(double* iVect, const double iV0, const double iV1,
                  const double iV2, const double iV3) {
    iVect[0] = iV0;  iVect[1] = iV1;  iVect[2] = iV2;  iVect[3] = iV3;
  }

  void operator()() {
    int publishPeriod = 1.0e6/mPublishFrequency;
    bot_core::pose_t msgHead;
    bot_core::rigid_transform_t msgHeadToBody;
    fillVector(msgHeadToBody.trans, -0.2, 0, -0.67);
    fillVector(msgHeadToBody.quat, 1,0,0,0);
    fillVector(msgHead.pos, 0,0,1.5);
    fillVector(msgHead.vel, 0,0,0);
    fillVector(msgHead.rotation_rate, 0,0,0);
    fillVector(msgHead.accel, 0,0,0);

    const double kPi = acos(-1);
    double rpy[] = {mRoll*kPi/180, mPitch*kPi/180, mYaw*kPi/180};
    bot_roll_pitch_yaw_to_quat(rpy, msgHead.orientation);
    
    auto lcm = mLcmWrapper->get();
    int64_t startTime, currentTime, remainingTime;

    while (true) {
      startTime = drc::Clock::instance()->getCurrentTime();
      msgHead.utime = msgHeadToBody.utime = startTime;
      lcm->publish("POSE_HEAD", &msgHead);
      lcm->publish("HEAD_TO_BODY", &msgHeadToBody);
      currentTime = drc::Clock::instance()->getCurrentTime();
      remainingTime = publishPeriod - (currentTime - startTime);
      if (remainingTime > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(remainingTime));
      }
    }
  }
};

int main(const int iArgc, const char** iArgv) {

  Publisher publisher;

  // parse command line args
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(publisher.mPublishFrequency, "f", "publish_frequency",
          "frequency of timestamp publications, in Hz");
  opt.add(publisher.mRoll, "r", "roll", "head roll in degrees");
  opt.add(publisher.mPitch, "p", "pitch", "head pitch in degrees");
  opt.add(publisher.mYaw, "y", "yaw", "head yaw in degrees");
  opt.parse();

  publisher.start();

  return 0;
}
