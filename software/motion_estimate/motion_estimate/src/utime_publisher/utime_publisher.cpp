#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include <ConciseArgs>
#include <lcm/lcm-cpp.hpp>
#include <bot_core/timestamp.h>
#include <drc_utils/LcmWrapper.hpp>

#include <lcmtypes/bot_core/rigid_transform_t.hpp>
#include <lcmtypes/drc/robot_state_t.hpp>
#include <lcmtypes/drc/utime_t.hpp>

struct Publisher {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  int mPublishFrequency;
  std::string mOutputChannel;
  std::thread mThread;
  bool mGotMessage;

  Publisher() {
    mPublishFrequency = 1000;
    mOutputChannel = "ROBOT_UTIME";
    mLcmWrapper.reset(new drc::LcmWrapper());
  }

  template<typename T>
  void handler(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
               const T* iMessage) {
    drc::utime_t msg;
    msg.utime = iMessage->utime;
    mLcmWrapper->get()->publish(mOutputChannel, &msg);
    mGotMessage = true;
  }

  template<typename T>
  bool tryChannel(const std::string& iChannel) {
    auto lcm = mLcmWrapper->get();

    mGotMessage = false;
    auto subscription = lcm->subscribe(iChannel, &Publisher::handler<T>, this);
    std::cout << "trying " << iChannel << "..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (mGotMessage) {
      std::cout << "using " << iChannel << " for timestamps" << std::endl;
      mLcmWrapper->stopHandleThread();
      return true;
    }
    lcm->unsubscribe(subscription);
    return false;
  }

  void start() {
    mLcmWrapper->startHandleThread(false);

    while (true) {
      if (tryChannel<drc::robot_state_t>("ROBOT_STATE") ||
          tryChannel<bot_core::rigid_transform_t>("PRE_SPINDLE_TO_POST_SPINDLE")) {
        mLcmWrapper->startHandleThread(true);
        return;
      }
      if (mPublishFrequency > 0) break;
    }

    std::cout << "no state messages found; publishing at " <<
      mPublishFrequency << " hz using wall time" << std::endl;
    mThread = std::thread(std::ref(*this));
    mThread.join();
  }

  void operator()() {
    int publishPeriod = 1.0e6/mPublishFrequency;
    drc::utime_t msg;
    auto lcm = mLcmWrapper->get();
    int64_t currentTime, remainingTime;

    while (true) {
      msg.utime = bot_timestamp_now();
      lcm->publish(mOutputChannel, &msg);
      currentTime = bot_timestamp_now();
      remainingTime = publishPeriod - (currentTime - msg.utime);
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
  opt.add(publisher.mOutputChannel, "o", "output_channel",
          "channel on which to publih utime messages");
  opt.parse();

  publisher.start();

  return 0;
}
