#ifndef _SIGNALHANDLER_H_
#define _SIGNALHANDLER_H_

#include <QObject>
#include <QHash>
#include <string>


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

  SignalHandler(const QString& channel);
  ~SignalHandler();

  SignalData* signalData()
  {
    return mSignalData;
  }

  virtual bool extractSignalData(const lcm::ReceiveBuffer* rbuf, const drc::robot_state_t* msg, float& timeNow, float& signalValue) = 0;

  void subscribe(lcm::LCM* lcmInstance);
  void unsubscribe(lcm::LCM* lcmInstance);

 protected:

  void handleRobotStateMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::robot_state_t* msg);

  SignalData* mSignalData;
  lcm::Subscription* mSubscription;

  QString mChannel;
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
#define declare_array_handler(className) \
class className : public SignalHandler \
{ \
public: \
  className(SignalDescription* desc); \
  virtual bool extractSignalData(const lcm::ReceiveBuffer* rbuf, const drc::robot_state_t* msg, float& timeNow, float& signalValue); \
  static QString messageType() { return "drc.robot_state_t"; } \
  static QString fieldName(); \
protected: \
  int mArrayIndex; \
};

declare_array_handler(RobotStateJointPositionHandler);
declare_array_handler(RobotStateJointVelocityHandler);
declare_array_handler(RobotStateJointEffortHandler);


#endif
