#ifndef _SIGNALHANDLER_H_
#define _SIGNALHANDLER_H_

#include <QObject>
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


//-----------------------------------------------------------------------------
#define declare_array_handler(className) \
class className : public SignalHandler \
{ \
public: \
  className(const QString& channel, int arrayIndex); \
  virtual bool extractSignalData(const lcm::ReceiveBuffer* rbuf, const drc::robot_state_t* msg, float& timeNow, float& signalValue); \
protected: \
  int mArrayIndex; \
};

declare_array_handler(RobotStateJointPositionHandler);
declare_array_handler(RobotStateJointVelocityHandler);
declare_array_handler(RobotStateMeasuredEffortHandler);


#endif
