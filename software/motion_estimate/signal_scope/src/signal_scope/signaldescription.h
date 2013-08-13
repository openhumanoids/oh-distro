#ifndef _SIGNALDESCRIPTION_H_
#define _SIGNALDESCRIPTION_H_

#include <QObject>

class SignalData;


class SignalDescription
{

public:

  SignalDescription();
  ~SignalDescription();

  QString mChannel;
  QString mMessageType;
  QString mFieldName;
  QList<QString> mArrayKeys;

 protected:

};

#endif
