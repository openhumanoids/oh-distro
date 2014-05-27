#ifndef _SIGNALHANDLER_H_
#define _SIGNALHANDLER_H_

#include "lcmsubscriber.h"
#include "signaldescription.h"

#include <QHash>
#include <string>


namespace lcm {
  class LCM;
  class ReceiveBuffer;
  class Subscription;
}

class SignalData;
class SignalDescription;


class SignalHandler : public LCMSubscriber
{
  Q_OBJECT

public:

  SignalHandler(const SignalDescription* signalDescription, QObject* parent=0);
  virtual ~SignalHandler();

  SignalData* signalData()
  {
    return mSignalData;
  }

  virtual QString description() = 0;
  QString channel() { return mDescription.mChannel; }
  SignalDescription* signalDescription() { return &mDescription; }

  virtual bool extractSignalData(const lcm::ReceiveBuffer* rbuf, double& timeNow, double& signalValue) = 0;

  virtual void subscribe(lcm::LCM* lcmInstance);

 protected:

  void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel);

  SignalData* mSignalData;
  SignalDescription mDescription;
};


class SignalHandlerFactory
{
public:
    template<typename T>
    void registerClass()
    {
      mConstructors[T::messageType()][T::fieldName()] = &constructorHelper<T>;
      mValidArrayKeys[T::messageType()][T::fieldName()] = T::validArrayKeys();
    }

    SignalHandler* createHandler(const SignalDescription* desc) const;

    static SignalHandlerFactory& instance();

    QList<QString> messageTypes() { return mConstructors.keys(); }
    QList<QString> fieldNames(const QString& messageType) { return mConstructors.value(messageType).keys(); }
    const QList<QList<QString> >& validArrayKeys(const QString& messageType, const QString& fieldName) { return mValidArrayKeys[messageType][fieldName]; }

private:
  typedef SignalHandler* (*Constructor)(const SignalDescription* desc);

  template<typename T>
  static SignalHandler* constructorHelper(const SignalDescription* desc)
  {
    return new T(desc);
  }

  QHash<QString, QHash<QString, Constructor> > mConstructors;
  QHash<QString, QHash<QString, QList<QList<QString> >  > > mValidArrayKeys;
};


//-----------------------------------------------------------------------------
#define declare_signal_handler(className) \
class className : public SignalHandler \
{ \
public: \
  className(const SignalDescription* desc); \
  virtual bool extractSignalData(const lcm::ReceiveBuffer* rbuf, double& timeNow, double& signalValue); \
  static QString messageType(); \
  static QString fieldName(); \
  static QList<QList<QString> > validArrayKeys(); \
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
