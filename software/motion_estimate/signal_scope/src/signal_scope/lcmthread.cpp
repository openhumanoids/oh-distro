#include "lcmthread.h"

#include "signalhandler.h"

#include <lcm/lcm-cpp.hpp>


void LCMThread::initLCM()
{
  if (mLCM)
  {
    return;
  }

  //std::string logFile = "file:///source/drc/logs/snippet.lcm";
  //std::string logFile = "file:///source/drc/logs/lcmlog-2013-08-01.00";
  //mLCM = new lcm::LCM(logFile);

  mLCM = new lcm::LCM();
  if (!mLCM->good())
  {
    printf("initLCM() failed.\n");
    return;
  }
}

void LCMThread::addSignalHandler(SignalHandler* handler)
{
  // mMutex.lock();
  this->initLCM();
  mSignalHandlers.append(handler);
  handler->subscribe(mLCM);
  // mMutex.unlock();
}

void LCMThread::removeSignalHandler(SignalHandler* handler)
{
  handler->unsubscribe(mLCM);
  mSignalHandlers.removeAll(handler);
}

void LCMThread::run()
{
  this->initLCM();

  while (!mShouldStop)
  {
    if (mShouldPause)
    {
      this->waitForResume();
    }

    // mMutex.lock();
    int result = mLCM->handle();
    // mMutex.unlock();
    if (result != 0)
    {
      printf("mLCM->handle() returned non-zero.  Terminating LCM thread.\n");
      break;
    }
  }
}

void LCMThread::stop()
{
  mShouldStop = true;
  this->resume();
}

void LCMThread::pause()
{
  mShouldPause = true;
}

void LCMThread::waitForResume()
{
  mMutex.lock();
  mWaitCondition.wait(&mMutex);
  mMutex.unlock();
}

void LCMThread::resume()
{
  mShouldPause = false;
  mWaitCondition.wakeAll();
}
