#ifndef _JOINTNAMES_H_
#define _JOINTNAMES_H_

#include <QString>
#include <QList>

class JointNames
{
public:

  JointNames();

  static int numberOfJoints();
  static int indexOfJointName(const QString& jointName);
  static QString jointName(int jointIndex);

  QList<QString> mNames;
};

#endif
