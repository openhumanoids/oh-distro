#ifndef _PYTHONCHANNELSUBSCRIBERCOLLECTION_H_
#define _PYTHONCHANNELSUBSCRIBERCOLLECTION_H_

#include "lcmthread.h"
#include "pythonchannelsubscriber.h"
#include "pythonsignalhandler.h"


#include <PythonQt.h>


class PythonChannelSubscriberCollection : public QObject
{
  Q_OBJECT

public:

  PythonChannelSubscriberCollection(LCMThread* lcmThread, PythonQtObjectPtr decodeCallback, QObject* parent=0) : QObject(parent)
  {
    mLCMThread = lcmThread;
    mDecodeCallback = decodeCallback;
  }

  void addSignalHandler(SignalHandler* handler)
  {
    PythonSignalHandler* pythonHandler = qobject_cast<PythonSignalHandler*>(handler);
    if (pythonHandler)
    {
      this->addSignalHandler(pythonHandler);
    }
    else
    {
      mLCMThread->addSubscriber(handler);
    }
  }

  void removeSignalHandler(SignalHandler* handler)
  {
    PythonSignalHandler* pythonHandler = qobject_cast<PythonSignalHandler*>(handler);
    if (pythonHandler)
    {
      this->removeSignalHandler(pythonHandler);
    }
    else
    {
      mLCMThread->removeSubscriber(handler);
    }
  }

  void addSignalHandler(PythonSignalHandler* handler)
  {
    QString channel = handler->channel();

    if (!mChannelSubscribers.contains(channel))
    {
      PythonChannelSubscriber* subscriber = new PythonChannelSubscriber(channel, mDecodeCallback, this);
      mChannelSubscribers[channel] = subscriber;
      mLCMThread->addSubscriber(subscriber);
    }

    PythonChannelSubscriber* subscriber = mChannelSubscribers[channel];
    subscriber->addSignalHandler(handler);
  }

  void removeSignalHandler(PythonSignalHandler* handler)
  {
    QString channel = handler->channel();
    PythonChannelSubscriber* subscriber = mChannelSubscribers.value(channel);
    if (!subscriber)
    {
      return;
    }

    subscriber->removeSignalHandler(handler);
  }

 protected:

  LCMThread* mLCMThread;
  PythonQtObjectPtr mDecodeCallback;
  QMap<QString, PythonChannelSubscriber* > mChannelSubscribers;
};


#endif
