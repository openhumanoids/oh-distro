#include "PointContactRelation.h"
#include <sstream>

using namespace action_authoring;

PointContactRelation::PointContactRelation() : RelationState(RelationState::POINT_CONTACT)
{
    _point1 = Eigen::Vector3f(0.0, 0.0, 0.0);
    _point2 = Eigen::Vector3f(0.0, 0.0, 0.0);
}

PointContactRelation::PointContactRelation(Eigen::Vector3f p1, Eigen::Vector3f p2) : RelationState(RelationState::POINT_CONTACT)
{
    _point1 = p1;
    _point2 = p2;
}

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
    return ss.str();
}
