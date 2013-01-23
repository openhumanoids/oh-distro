#ifndef MANIPULATION_RELATION_H
#define MANIPULATION_RELATION_H

#include "boost/shared_ptr.hpp"
#include "affordance/AffordanceState.h"
#include "RelationState.h"
#include "ManipulatorState.h"

namespace action_authoring {
    class ManipulationRelation {

    //----------Constructor
    public:
	MainipulationRelation(AffPtr affordance, ManPtr manipulator, RelationStatePtr relationState);

    //----------Accessors
	AffPtr GetAffordance();
	ManPtr GetManipulator();
	RelationStatePtr GetRelationState();

	void SetAffordance(AffPtr affordance);
	void SetManipulator(ManPtr manipulator);
	void SetRelationState(RelationStatePtr relationState);

    //------------Fields
    private:
	AffPtr _affordance;
	ManPtr _manipulator;
	RelationStatePtr _relationState;

    }; // class ManipulationRelation
    typedef boost::shared_ptr<ManipulationRelation> ManRelPtr;
    typedef boost::shared_ptr<const ManipulationRelation> ManRelConstPtr;

} //namespace action_authoring

#endif //MANIPULATION_RELATION_H
