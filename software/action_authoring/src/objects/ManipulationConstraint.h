#ifndef MANIPULATION_RELATION_H
#define MANIPULATION_RELATION_H

#include <boost/shared_ptr.hpp>
#include <affordance/AffordanceState.h>
#include <affordance/ManipulatorState.h>
#include "RelationState.h"
#include "AtomicConstraint.h"

namespace action_authoring
{
class ManipulationConstraint : public AtomicConstraint
{

    //----------Constructor
public:
    ManipulationConstraint(affordance::AffConstPtr affordance,
                         affordance::ManipulatorStateConstPtr manipulator,
                         RelationStatePtr relationState);

    ManipulationConstraint(affordance::AffConstPtr affordance,
                         affordance::ManipulatorStateConstPtr manipulator,
                         RelationStatePtr relationState,
                         double timeLowerBound,
                         double timeUpperBound
                         );

    //----------Accessors
    affordance::AffConstPtr getAffordance() const
    {
        return _affordance;
    }
    affordance::ManipulatorStateConstPtr getManipulator() const
    {
        return _manipulator;
    }
    RelationStatePtr getRelationState() const
    {
        return _relationState;
    }
    double getTimeLowerBound() const
    {
        return _timeLowerBound;
    }
    double getTimeUpperBound() const
    {
        return _timeUpperBound;
    }

    void setAffordance(affordance::AffConstPtr affordance)
    {
        _affordance = affordance;
    }
    void setManipulator(affordance::ManipulatorStateConstPtr manipulator)
    {
        _manipulator = manipulator;
    }
    void setRelationState(RelationStatePtr relationState)
    {
        _relationState = relationState;
    }
    void setTimeLowerBound(double timeLowerBound)
    {
        _timeLowerBound = timeLowerBound;
    }
    void setTimeUpperBound(double timeUpperBound)
    {
        _timeUpperBound = timeUpperBound;
    }

    drc::contact_goal_t toLCM();

    //------------Fields
private:
    affordance::AffConstPtr _affordance;
    affordance::ManipulatorStateConstPtr _manipulator;
    RelationStatePtr _relationState;

}; // class ManipulationConstraint
typedef boost::shared_ptr<ManipulationConstraint> ManRelPtr;
typedef boost::shared_ptr<const ManipulationConstraint> ManRelConstPtr;

} //namespace action_authoring

#endif //MANIPULATION_RELATION_H
