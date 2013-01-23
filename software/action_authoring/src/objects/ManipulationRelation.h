#ifndef MANIPULATION_RELATION_H
#define MANIPULATION_RELATION_H

#include <boost/shared_ptr.hpp>
#include <affordance/AffordanceState.h>
#include <affordance/ManipulatorState.h>
#include "RelationState.h"


namespace action_authoring {
    class ManipulationRelation {

    //----------Constructor
    public:
        ManipulationRelation(affordance::AffPtr affordance, affordance::ManipulatorStatePtr manipulator, RelationStatePtr relationState);

    //----------Accessors
        affordance::AffPtr getAffordance() { return _affordance; }
        affordance::ManipulatorStatePtr getManipulator() { return _manipulator; }
	RelationStatePtr getRelationState() { return _relationState; }

        void setAffordance(affordance::AffPtr affordance) { _affordance = affordance; }
        void setManipulator(affordance::ManipulatorStatePtr manipulator) { _manipulator = manipulator; }
	void setRelationState(RelationStatePtr relationState) { _relationState = relationState; }

    //------------Fields
    private:
        affordance::AffPtr _affordance;
        affordance::ManipulatorStatePtr _manipulator;
	RelationStatePtr _relationState;

    }; // class ManipulationRelation
    typedef boost::shared_ptr<ManipulationRelation> ManRelPtr;
    typedef boost::shared_ptr<const ManipulationRelation> ManRelConstPtr;

} //namespace action_authoring

#endif //MANIPULATION_RELATION_H
