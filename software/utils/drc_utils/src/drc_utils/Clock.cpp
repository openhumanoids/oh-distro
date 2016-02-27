#include "Clock.hpp"

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <bot_core/timestamp.h>
#include <lcmtypes/bot_core/utime_t.hpp>
#include <lcmtypes/bot_core/robot_state_t.hpp>

using namespace drc;

struct Clock::Impl {
  std::shared_ptr<lcm::LCM> mLcm;
  std::string mChannel;
  int mTimeoutInterval;
  bool mUseTimeMessages;
  bool mUseRobotState;
  bool mUseRealTimeWhenInvalid;
  bool mVerbose;

  int64_t mCurrentTime;
  lcm::Subscription* mTimeSubscription;
  int64_t mLastMessageReceivedTime;

  Impl() {
    mTimeoutInterval = 2000;
    if (false) {
      mChannel = "ROBOT_UTIME";
      mUseRobotState = false;
    } else {
      mChannel = "EST_ROBOT_STATE";
      mUseRobotState = true;
    }
    mLcm.reset(new lcm::LCM());
    mUseTimeMessages = true;
    mUseRealTimeWhenInvalid = true;
    mCurrentTime = 0;
    mTimeSubscription = NULL;
    mLastMessageReceivedTime = 0;
    mVerbose = true;

    subscribe();
  }

  ~Impl() {
    unsubscribe();
  }

  void subscribe() {
    unsubscribe();
    if ((mLcm != NULL) && mUseTimeMessages) {
      if (mUseRobotState) {
        mTimeSubscription = mLcm->subscribe(mChannel, &Impl::onState, this);
      }
      else {
        mTimeSubscription = mLcm->subscribe(mChannel, &Impl::onTime, this);
      }
    }
  }

  void unsubscribe() {
    if ((mLcm != NULL) && (mTimeSubscription != NULL)) {
      mLcm->unsubscribe(mTimeSubscription);
      mTimeSubscription = NULL;
    }
  }

  void setLcm(const std::shared_ptr<lcm::LCM>& iLcm) {
    unsubscribe();
    mLcm = iLcm;
    subscribe();
  }

  void onTime(const lcm::ReceiveBuffer* iBuf,
              const std::string& iChannel,
              const bot_core::utime_t* iMessage) {
    mCurrentTime = iMessage->utime;
    int64_t curRealTime = bot_timestamp_now();
    int64_t dt = curRealTime - mLastMessageReceivedTime;
    if ((mLastMessageReceivedTime > 0) && (dt > mTimeoutInterval*1000)) {
      if (mVerbose) {
        std::cout << "drc::Clock: WARNING: last timestamp message received " <<
          (dt/1e6) << " seconds ago" << std::endl;
      }
    }
    mLastMessageReceivedTime = curRealTime;
  }

  void onState(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const bot_core::robot_state_t* iMessage) {
    mCurrentTime = iMessage->utime;
    int64_t curRealTime = bot_timestamp_now();
    int64_t dt = curRealTime - mLastMessageReceivedTime;
    if ((mLastMessageReceivedTime > 0) && (dt > mTimeoutInterval*1000)) {
      if (mVerbose) {
        std::cout << "drc::Clock: WARNING: last timestamp message received " <<
          (dt/1e6) << " seconds ago" << std::endl;
      }
    }
    mLastMessageReceivedTime = curRealTime;
  }

};


Clock::
Clock() {
  mImpl.reset(new Impl());
}

Clock::
~Clock() {
}

Clock* Clock::
instance() {
  static Clock theClock;
  return &theClock;
}

void Clock::
setLcm(const std::shared_ptr<lcm::LCM>& iLcm) {
  mImpl->setLcm(iLcm);
}

void Clock::
setLcm(const lcm_t* iLcm) {
  mImpl->setLcm(std::shared_ptr<lcm::LCM>(new lcm::LCM((lcm_t*)iLcm)));
}

void Clock::
setChannel(const std::string& iChannelName) {
  mImpl->mChannel = iChannelName;
  mImpl->subscribe();
}

std::string Clock::
getChannel() const {
  return mImpl->mChannel;
}

void Clock::
setTimeoutInterval(const int iMilliseconds) {
  mImpl->mTimeoutInterval = iMilliseconds;
}

void Clock::
useTimeMessages(const bool iVal) {
  mImpl->mUseTimeMessages = iVal;
  mImpl->subscribe();
}

void Clock::
useRealTimeWhenInvalid(const bool iVal) {
  mImpl->mUseRealTimeWhenInvalid = iVal;
}

void Clock::
setVerbose(const bool iVal) {
  mImpl->mVerbose = iVal;
}

int64_t Clock::
getCurrentTime() const {
  int64_t curRealTime = bot_timestamp_now();
  if (!mImpl->mUseTimeMessages) {
    return curRealTime;
  }
  if (mImpl->mCurrentTime > 0) {
    int64_t dt = curRealTime - mImpl->mLastMessageReceivedTime;
    if (dt > mImpl->mTimeoutInterval*1000) {
      if (mImpl->mVerbose) {
        std::cout << "drc::Clock: WARNING: last timestamp message " <<
          "received " << (dt/1e6) << " seconds ago" << std::endl;
      }
    }
    return mImpl->mCurrentTime;
  }
  else {
    return mImpl->mUseRealTimeWhenInvalid ? curRealTime : -1;
  }
}

int64_t Clock::
getCurrentWallTime() const {
  return bot_timestamp_now();
}
