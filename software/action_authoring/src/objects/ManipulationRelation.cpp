#include "ManipulationRelation.h"

using namespace action_authoring;
using namespace affordance;

MainipulationRelation::
MainipulationRelation(AffPtr affordance, ManPtr manipulator, 
		      RelationStatePtr relationState) {
    _affordance = affordance;
    _manipulator = manipulator;
    _relationState = relationState;
}

AffPtr
ManipulationRelation::
GetAffordance() {
    return _affordance;
}

ManPtr
ManipulationRelation::
GetManipulator() {
    return _manipulator;
}

RelationStatePtr
ManipulationRelation::
GetRelationState() {
    return _relationState;
}

void
ManipulationRelation::
SetAffordance(AffPtr affordance) {
    _affordance = affordance;
}

void
ManipulationRelation::
SetManipulator(ManPtr manipulator) {
    _manipulator = manipulator; 
}

void
ManipulationRelation::
SetRelationState(RelationStatePtr relationState) {
    _relationState = relationState; 
}

