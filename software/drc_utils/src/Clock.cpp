#include "Clock.hpp"

#include <iostream>
#include <bot_core/timestamp.h>
#include <lcmtypes/drc/utime_t.hpp>

using namespace drc;

namespace drc {

// helper class; concrete implementation
class ClockImpl : public Clock {
public:
  ClockImpl() {
    mCurrentTime = 0;
    mTimeSubscription = NULL;
    mLastMessageReceivedTime = 0;
    update();
  }

  ~ClockImpl() {
    if (mTimeSubscription != NULL) {
      mLcm->unsubscribe(mTimeSubscription);
    }
  }

  int64_t getCurrentTime() const {
    int64_t curRealTime = bot_timestamp_now();
    if (!mUseTimeMessages) {
      return curRealTime;
    }
    if (mCurrentTime > 0) {
      int64_t dt = curRealTime - mLastMessageReceivedTime;
      if (dt > mTimeoutInterval*1000) {
        std::cout << "drc::Clock: WARNING: last timestamp message received " <<
          (dt/1e6) << " seconds ago" << std::endl;        
      }
      return mCurrentTime;
    }
    else {
      return mUseRealTimeWhenInvalid ? curRealTime : -1;
    }
  }

  void onTime(const lcm::ReceiveBuffer* iBuf,
              const std::string& iChannel,
              const drc::utime_t* iMessage) {
    mCurrentTime = iMessage->utime;
    int64_t curRealTime = bot_timestamp_now();
    int64_t dt = curRealTime - mLastMessageReceivedTime;
    if ((mLastMessageReceivedTime > 0) && (dt > mTimeoutInterval*1000)) {
      std::cout << "drc::Clock: WARNING: last timestamp message received " <<
        (dt/1e6) << " seconds ago" << std::endl;
    }
    mLastMessageReceivedTime = curRealTime;
  }

protected:
  void update() {
    if (mTimeSubscription != NULL) {
      mLcm->unsubscribe(mTimeSubscription);
    }
    if (mUseTimeMessages) {
      mTimeSubscription = mLcm->subscribe(mChannel, &ClockImpl::onTime, this);
    }
  }

protected:
  int64_t mCurrentTime;
  lcm::Subscription* mTimeSubscription;
  int64_t mLastMessageReceivedTime;
};

}

Clock::
Clock() {
  mTimeoutInterval = 2000;
  mChannel = "ROBOT_UTIME";  
  mLcm.reset(new lcm::LCM());
  mUseTimeMessages = true;
  mUseRealTimeWhenInvalid = true;
}

Clock::
~Clock() {
}

Clock* Clock::
instance() {
  static ClockImpl theClock;
  return &theClock;
}

void Clock::
setLcm(const boost::shared_ptr<lcm::LCM>& iLcm) {
  mLcm = iLcm;
  update();
}

void Clock::
setLcm(const lcm_t* iLcm) {
  mLcm.reset(new lcm::LCM((lcm_t*)iLcm));
  update();
}

void Clock::
setChannel(const std::string& iChannelName) {
  mChannel = iChannelName;
  update();
}

std::string Clock::
getChannel() const {
  return mChannel;
}

void Clock::
setTimeoutInterval(const int iMilliseconds) {
  mTimeoutInterval = iMilliseconds;
}

void Clock::
useTimeMessages(const bool iVal) {
  mUseTimeMessages = iVal;
  update();
}

void Clock::
useRealTimeWhenInvalid(const bool iVal) {
  mUseRealTimeWhenInvalid = iVal;
}
