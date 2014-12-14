#include "PointContactRelation.h"
#include <sstream>

using namespace action_authoring;
using namespace std;

typedef PointContactRelation PCR;

string PCR::EQUAL_STR = "=";
string PCR::LESS_THAN_STR = "<";
string PCR::GREATER_THAN_STR = ">";
string PCR::UNDEFINED_STR = "UNDEFINED";

string PCR::typeToStr(const PointContactRelation::InequalityType &type)
{
  switch(type)
    {
    case PCR::EQUAL:
      return EQUAL_STR;
    case PCR::LT:
      return LESS_THAN_STR;
    case PCR::GT:
      return GREATER_THAN_STR;
    case PCR::UNDEFINED:
      return UNDEFINED_STR;
    default:
      throw runtime_error("PointContactRelation: unknown type passed to typeToStr");
    }
}

PointContactRelation::InequalityType PointContactRelation::strToType(const std::string &s)
{
  if (s == LESS_THAN_STR)
    return PointContactRelation::LT;
  if (s == GREATER_THAN_STR)
    return PointContactRelation::GT;
  if (s == EQUAL_STR)
    return PointContactRelation::EQUAL;
  if (s == UNDEFINED_STR)
    return PointContactRelation::UNDEFINED;

  throw std::runtime_error("PointContactRelation: can't parse string: " + s);
}


PointContactRelation::PointContactRelation(Eigen::Vector3f p1, Eigen::Vector3f p2)
  : RelationState(RelationState::POINT_CONTACT),
    _point1(p1),
    _point2(p2),
    _x_relation(PointContactRelation::UNDEFINED),
    _y_relation(PointContactRelation::UNDEFINED),
    _z_relation(PointContactRelation::UNDEFINED),
    _offset(Eigen::Vector3f(0,0,0))
{ }



std::string
PointContactRelation::
getPrompt() const
{
    if (_point1.x() == 0 && _point1.y() == 0 && _point1.z() == 0)
    {
        return "click to bind point 1";
    }

    if (_point2.x() == 0 && _point2.y() == 0 && _point2.z() == 0)
    {
        return "click to bind point 2";
    }

    return "both points bound";
}

std::string
PointContactRelation::
getState() const
{
    std::stringstream ss;
    ss << " point 1 : (" << _point1.x() << ", " << _point1.y() << ", " << _point1.z() << ") ";
    ss << " point 2 : (" << _point2.x() << ", " << _point2.y() << ", " << _point2.z() << ") ";
    ss << " offset : " << _offset 
       <<  "; (x : " << _x_relation << " y : " << _y_relation << " z: " << _z_relation << ")";
    return ss.str();
}
