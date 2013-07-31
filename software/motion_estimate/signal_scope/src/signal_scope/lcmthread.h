#ifndef _LCMTHREAD_H_
#define _LCMTHREAD_H_

#include <QThread>
#include <QMutex>
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
    mLCM = 0;
  }

  void stop()
  {
    mShouldStop = true;
  }

  void addSignalHandler(SignalHandler* handler);
  void removeSignalHandler(SignalHandler* handler);

 protected:

  void run();
  void initLCM();

  bool mShouldStop;
  QList<SignalHandler*> mSignalHandlers;
  lcm::LCM* mLCM;

  QMutex mMutex;
};

#endif
