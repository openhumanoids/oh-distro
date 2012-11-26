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

  void setLcm(const boost::shared_ptr<lcm::LCM>& iLcm) {
    Clock::setLcm(iLcm);
    update();
  }

  void setChannel(const std::string& iChannel) {
    Clock::setChannel(iChannel);
    update();
  }

  int64_t getCurrentTime() const {
    int64_t curRealTime = bot_timestamp_now();
    if (mCurrentTime > 0) {
      int64_t dt = curRealTime - mLastMessageReceivedTime;
      if (dt > mTimeoutInterval*1000) {
        std::cout << "drc::Clock: WARNING: last timestamp message received " <<
          (dt/1e6) << " seconds ago" << std::endl;        
      }
      return mCurrentTime;
    }
    else {
      return curRealTime;
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
    mTimeSubscription = mLcm->subscribe(mChannel, &ClockImpl::onTime, this);
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
}

void Clock::
setChannel(const std::string& iChannelName) {
  mChannel = iChannelName;
}

std::string Clock::
getChannel() const {
  return mChannel;
}

void Clock::
setTimeoutInterval(const int iMilliseconds) {
  mTimeoutInterval = iMilliseconds;
}
