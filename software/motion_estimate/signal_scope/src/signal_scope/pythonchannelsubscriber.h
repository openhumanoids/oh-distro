#ifndef _PYTHONCHANNELSUBSCRIBER_H_
#define _PYTHONCHANNELSUBSCRIBER_H_

#include "lcmsubscriber.h"
#include "lcmthread.h"
#include "pythonsignalhandler.h"

#include <PythonQt.h>


class PythonChannelSubscriber : public LCMSubscriber
{
  Q_OBJECT

public:

  PythonChannelSubscriber(QString channel, PythonQtObjectPtr decodeCallback, QObject* parent=0) : LCMSubscriber(parent)
  {
    mChannel = channel;
    mDecodeCallback = decodeCallback;
  }

  QString channel()
  {
    return mChannel;
  }

  void subscribe(lcm::LCM* lcmHandle)
  {
    if (mSubscription)
    {
      printf("error: subscribe() called without first calling unsubscribe.\n");
      return;
    }
    mSubscription = lcmHandle->subscribe(this->channel().toAscii().data(), &PythonChannelSubscriber::handleMessageOnChannel, this);
  }


  void addSignalHandler(PythonSignalHandler* handler)
  {
    mHandlers.append(handler);
  }

  void removeSignalHandler(PythonSignalHandler* handler)
  {
    mHandlers.removeAll(handler);
  }

  QVariant decodeMessage(const lcm::ReceiveBuffer* rbuf)
  {
    if (!mDecodeCallback)
    {
      return QVariant();
    }

    QVariantList args;
    args << QByteArray((char*)rbuf->data, rbuf->data_size);

    return PythonQt::self()->call(mDecodeCallback, args);
  }


  void handleMessageOnChannel(const lcm::ReceiveBuffer* rbuf, const std::string& channel)
  {
    //printf("handle message on channel: %s\n", channel.c_str());

    QString channelStr = channel.c_str();

    if (!mHandlers.size())
    {
      return;
    }

    QVariant decodedMessage = this->decodeMessage(rbuf);
    if (!decodedMessage.isValid())
    {
      printf("decoded message is not valid\n");
      return;
    }


    foreach (PythonSignalHandler* handler, mHandlers)
    {
      handler->onNewMessage(decodedMessage);
    }
  }


  QString mChannel;
  PythonQtObjectPtr mDecodeCallback;
  QList<PythonSignalHandler*> mHandlers;
};

#endif
