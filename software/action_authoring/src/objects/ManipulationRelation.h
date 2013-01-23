#ifndef MANIPULATION_RELATION_H
#define MANIPULATION_RELATION_H

#include "boost/shared_ptr.hpp"
#include "affordance/AffordanceState.h"
#include "RelationState.h"
#include "ManipulatorState.h"

namespace action_authoring {
    class ManipulationRelation {

    public:
	MainipulationRelation(AffPtr affordance, ManPtr manipulator, RelationStatePtr relationState);
	AffPtr GetAffordance();
	ManPtr GetManipulator();
	RelationStatePtr GetRelationState();

	void SetAffordance(AffPtr affordance);
	void SetManipulator(ManPtr manipulator);
	void SetRelationState(RelationStatePtr relationState);

    protected:
	AffPtr _affordance;
	ManPtr _manipulator;
	RelationStatePtr _relationState;

    }
}

#endif //MANIPULATION_RELATION_H
