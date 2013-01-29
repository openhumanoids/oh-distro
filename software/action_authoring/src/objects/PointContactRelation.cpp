#include "PointContactRelation.h"

using namespace action_authoring;

PointContactRelation::PointContactRelation() : RelationState(RelationState::POINT_CONTACT) {

}

std::string
PointContactRelation::
getPrompt() const {
    return "point test";
    if (_point1.x() == 0)
	return "click to bind point 1";
    if (_point2.x() == 0) 
	return "click to bind point 2";
    return "both points bound";
}

std::string
PointContactRelation::
getState() const {
    return "there we go";
	//return (_point1 == NULL ? 
}
