#include "LcmWrapper.hpp"

#include <iostream>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include <sys/select.h>

using namespace drc;

struct LcmWrapper::Helper {
  bool mIsRunning;
  bool mIsJoined;
  std::thread mHandleThread;
  std::shared_ptr<lcm::LCM> mLcm;

  Helper() {
    mIsRunning = false;
    mIsJoined = false;
  }

  void operator()() {
    while(mIsRunning) {
      int fn = mLcm->getFileno();
      fd_set input_set;
      FD_ZERO(&input_set);
      FD_SET(fn, &input_set);
      struct timeval timeout = { 0, 200*1000 };
      int status = select(fn+1, &input_set, NULL, NULL, &timeout);
      if (status == 0) {}
      else if (status < 0) break;
      else if (FD_ISSET(fn, &input_set)) {
        if (0 != mLcm->handle()) break;
      }
    }
  }
};

LcmWrapper::
LcmWrapper() {
  mHelper.reset(new Helper());
  mHelper->mLcm.reset(new lcm::LCM());
}

LcmWrapper::
LcmWrapper(const lcm_t* iLcm) {
  mHelper.reset(new Helper());
  mHelper->mLcm.reset(new lcm::LCM(const_cast<lcm_t*>(iLcm)));
}

LcmWrapper::
LcmWrapper(const std::shared_ptr<lcm::LCM>& iLcm) {
  mHelper.reset(new Helper());
  mHelper->mLcm = iLcm;
}

LcmWrapper::
~LcmWrapper() {
  stopHandleThread();
}

bool LcmWrapper::
startHandleThread(const bool iJoined) {
  if (mHelper->mIsRunning) return false;
  mHelper->mIsRunning = true;
  mHelper->mIsJoined = iJoined;
  mHelper->mHandleThread = std::thread(std::ref(*mHelper));
  std::cout << "Started lcm handle loop" << std::endl;
  if (iJoined) mHelper->mHandleThread.join();
  return true;
}

bool LcmWrapper::
stopHandleThread() {
  if (!mHelper->mIsRunning) return false;
  mHelper->mIsRunning = false;
  if (!mHelper->mIsJoined && mHelper->mHandleThread.joinable()) {
    mHelper->mHandleThread.join();
  }
  return true;
}

bool LcmWrapper::
isThreadRunning() const {
  return mHelper->mIsRunning;
}

std::shared_ptr<lcm::LCM> LcmWrapper::
get() const {
  return mHelper->mLcm;
}

lcm_t* LcmWrapper::
getC() const {
  return mHelper->mLcm->getUnderlyingLCM();
}
