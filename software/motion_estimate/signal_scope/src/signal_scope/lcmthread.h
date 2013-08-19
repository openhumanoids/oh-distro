#ifndef _LCMTHREAD_H_
#define _LCMTHREAD_H_

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <string>

class SignalHandler;

namespace lcm
{
  class LCM;
}

class LCMThread : public QThread
 {
  Q_OBJECT

public:

  LCMThread()
  {
    mShouldStop = false;
    mShouldPause = false;
    mLCM = 0;
  }

  void stop();
  void pause();
  void resume();

  void addSignalHandler(SignalHandler* handler);
  void removeSignalHandler(SignalHandler* handler);

 protected:

  void run();
  void initLCM();
  void waitForResume();

  bool mShouldPause;
  bool mShouldStop;
  QList<SignalHandler*> mSignalHandlers;
  lcm::LCM* mLCM;

  QMutex mMutex;
  QWaitCondition mWaitCondition;
};

#endif
