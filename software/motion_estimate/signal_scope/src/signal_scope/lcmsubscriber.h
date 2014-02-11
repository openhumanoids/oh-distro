#ifndef _LCMSUBSCRIBER_h
#define _LCMSUBSCRIBER_h

#include <QObject>

#include <lcm/lcm-cpp.hpp>

namespace lcm
{
  class LCM;
}

class LCMSubscriber : public QObject
{
  Q_OBJECT

public:

  LCMSubscriber(QObject* parent=0) : QObject(parent)
  {
    mSubscription = 0;
  }

  virtual ~LCMSubscriber()
  {

  }

  virtual void subscribe(lcm::LCM* lcmInstance) = 0;

  virtual void unsubscribe(lcm::LCM* lcmHandle)
  {
    lcmHandle->unsubscribe(mSubscription);
    mSubscription = 0;
  }

 protected:

  lcm::Subscription* mSubscription;
};

#endif
