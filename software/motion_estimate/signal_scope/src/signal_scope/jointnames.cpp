#include "jointnames.h"

namespace
{
  JointNames instance;
}

JointNames::JointNames()
{
  mNames
    << "BACK_BKZ"
    << "BACK_BKY"
    << "BACK_BKX"
    << "NECK_AY"
    << "L_LEG_HPZ"
    << "L_LEG_HPX"
    << "L_LEG_HPY"
    << "L_LEG_KNY"
    << "L_LEG_AKY"
    << "L_LEG_AKX"
    << "R_LEG_HPZ"
    << "R_LEG_HPX"
    << "R_LEG_HPY"
    << "R_LEG_KNY"
    << "R_LEG_AKY"
    << "R_LEG_AKX"
    << "L_ARM_USY"
    << "L_ARM_SHX"
    << "L_ARM_ELY"
    << "L_ARM_ELX"
    << "L_ARM_UWY"
    << "L_ARM_MWX"
    << "R_ARM_USY"
    << "R_ARM_SHX"
    << "R_ARM_ELY"
    << "R_ARM_ELX"
    << "R_ARM_UWY"
    << "R_ARM_MWX";
}

int JointNames::numberOfJoints()
{
  return instance.mNames.length();
}

int JointNames::indexOfJointName(const QString& jointName)
{
  return instance.mNames.indexOf(jointName);
}

QString JointNames::jointName(int jointIndex)
{
  if (jointIndex < numberOfJoints())
  {
    return instance.mNames[jointIndex];
  }

  return QString();
}

const QList<QString>& JointNames::jointNames()
{
  return instance.mNames;
}
