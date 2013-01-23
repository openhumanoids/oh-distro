#include "ManipulationRelation.h"

using namespace action_authoring;
using namespace affordance;

ManipulationRelation::
ManipulationRelation(AffPtr affordance, affordance::ManipulatorStatePtr manipulator, 
		      RelationStatePtr relationState) {
    _affordance = affordance;
    _manipulator = manipulator;
    _relationState = relationState;
}
