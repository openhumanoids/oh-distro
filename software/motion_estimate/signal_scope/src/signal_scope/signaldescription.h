#ifndef _SIGNALDESCRIPTION_H_
#define _SIGNALDESCRIPTION_H_

#include <QObject>
#include <QColor>

class SignalData;


class SignalDescription
{

public:

  SignalDescription();
  ~SignalDescription();

  QString mChannel;
  QString mMessageType;
  QString mFieldName;
  QColor mColor;
  QList<QString> mArrayKeys;

  QString descriptionString() const
  {
    QStringList args;
    args << mChannel << mMessageType << mFieldName << mArrayKeys;
    return args.join(".");
  }

 protected:

};

#endif
