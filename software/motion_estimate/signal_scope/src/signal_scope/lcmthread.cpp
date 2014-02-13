#include "lcmthread.h"
#include "lcmsubscriber.h"

#include <QMutexLocker>
#include <lcm/lcm-cpp.hpp>

LCMThread::LCMThread()
{
  mShouldStop = false;
  mShouldPause = false;
  mLCM = 0;
}

LCMThread::~LCMThread()
{
}

void LCMThread::initLCM()
{
  QMutexLocker locker(&mMutex);

  if (mLCM)
  {
    return;
  }

  mLCM = new lcm::LCM();
  if (!mLCM->good())
  {
    printf("initLCM() failed.\n");
    return;
  }
}

void LCMThread::addSubscriber(LCMSubscriber* subscriber)
{
  this->initLCM();
  mSubscribers.append(subscriber);
  subscriber->subscribe(mLCM);
}

void LCMThread::removeSubscriber(LCMSubscriber* subscriber)
{
  this->initLCM();
  subscriber->unsubscribe(mLCM);
  mSubscribers.removeAll(subscriber);
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

    int result = mLCM->handle();
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
