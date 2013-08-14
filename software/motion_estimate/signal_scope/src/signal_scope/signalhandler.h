#ifndef _SIGNALHANDLER_H_
#define _SIGNALHANDLER_H_

#include <QObject>
#include <QHash>
#include <string>

#include "signaldescription.h"

namespace lcm {
  class LCM;
  class ReceiveBuffer;
  class Subscription;
}

namespace drc {
  class robot_state_t;
}

class SignalData;
class SignalDescription;


class SignalHandler : public QObject
{
  Q_OBJECT

public:

  SignalHandler(SignalDescription* signalDescription);
  ~SignalHandler();

  SignalData* signalData()
  {
    return mSignalData;
  }

  virtual QString description() = 0;
  QString channel() { return mDescription.mChannel; }
  SignalDescription* signalDescription() { return &mDescription; }

  virtual bool extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) = 0;

  void subscribe(lcm::LCM* lcmInstance);
  void unsubscribe(lcm::LCM* lcmInstance);

 protected:

  void handleRobotStateMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel);

  SignalData* mSignalData;
  lcm::Subscription* mSubscription;

  SignalDescription mDescription;
};


class SignalHandlerFactory
{
public:
    template<typename T>
    void registerClass()
    {
      mConstructors[T::messageType()][T::fieldName()] = &constructorHelper<T>;
    }

    SignalHandler* createHandler(SignalDescription* desc) const;

    static SignalHandlerFactory& instance();

    QList<QString> messageTypes() { return mConstructors.keys(); }
    QList<QString> fieldNames(const QString& messageType) { return mConstructors.value(messageType).keys(); }

private:
  typedef SignalHandler* (*Constructor)(SignalDescription* desc);

  template<typename T>
  static SignalHandler* constructorHelper(SignalDescription* desc)
  {
    return new T(desc);
  }

  QHash<QString, QHash<QString, Constructor> > mConstructors;
};


//-----------------------------------------------------------------------------
#define declare_signal_handler(className) \
class className : public SignalHandler \
{ \
public: \
  className(SignalDescription* desc); \
  virtual bool extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue); \
  static QString messageType(); \
  static QString fieldName(); \
  virtual QString description(); \
protected: \
  int mArrayIndex; \
  int mArrayIndex2; \
  QString mArrayKey; \
  QString mArrayKey2; \
};


/*
declare_signal_handler(RobotStateJointPositionHandler);
declare_signal_handler(RobotStateJointVelocityHandler);
declare_signal_handler(RobotStateJointEffortHandler);

declare_signal_handler(RobotStatePoseTranslationXHandler);
declare_signal_handler(RobotStatePoseTranslationYHandler);
declare_signal_handler(RobotStatePoseTranslationZHandler);

declare_signal_handler(RobotStatePoseRotationWHandler);
declare_signal_handler(RobotStatePoseRotationXHandler);
declare_signal_handler(RobotStatePoseRotationYHandler);
declare_signal_handler(RobotStatePoseRotationZHandler);

*/



#endif
