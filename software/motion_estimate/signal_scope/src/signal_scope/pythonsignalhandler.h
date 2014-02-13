#ifndef _PYTHONSIGNALHANDLER_H_
#define _PYTHONSIGNALHANDLER_H_

#include "signalhandler.h"
#include "signaldescription.h"
#include "signaldata.h"

#include <PythonQt.h>

class SignalData;
class SignalDescription;


namespace
{
  uint64_t timeOffset = 0;
}


class PythonSignalHandler : public SignalHandler
{
  Q_OBJECT

public:

  PythonSignalHandler(SignalDescription* signalDescription, PythonQtObjectPtr callback) : SignalHandler(signalDescription)
  {
    mCallback = callback;
  }

  virtual QString description()
  {
    return mDescription.descriptionString();
  }

  void onNewMessage(const QVariant& message)
  {
    float timeNow;
    float signalValue;

    bool valid = this->extractSignalData(message, timeNow, signalValue);
    if (valid)
    {
      mSignalData->appendSample(timeNow, signalValue);
    }
    else
    {
      mSignalData->flagMessageError();
    }
  }

  virtual bool extractSignalData(const QVariant& message, float& timeNow, float& signalValue)
  {
    //printf("calling python handler...\n");

    QVariantList args;
    args << message;

    QVariant result = PythonQt::self()->call(mCallback, args);
    QList<QVariant> values = result.toList();
    if (values.size() == 2)
    {

      quint64 msgUtime = values[0].toULongLong();

      if (timeOffset == 0) timeOffset = msgUtime;
      timeNow = (msgUtime - timeOffset)/1e6;
      signalValue = values[1].toFloat();

      //printf("%f, %f\n", timeNow, signalValue);
      return true;
    }
    else
    {
      return false;
    }

    return true;
  }

  virtual bool extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue)
  {
    return false;
  }

 protected:

  PythonQtObjectPtr mCallback;
};


#endif
