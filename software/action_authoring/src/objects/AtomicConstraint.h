#ifndef ATOMIC_CONSTRAINT_H
#define ATOMIC_CONSTRAINT_H

#include <boost/shared_ptr.hpp>
#include <affordance/AffordanceState.h>
#include <affordance/ManipulatorState.h>
#include <action_authoring/RelationState.h>

namespace action_authoring
{

/**todo: add comment
todo: make some pure virtual methods*/

class AtomicConstraint
{

    //---------------Accessors
public:
    virtual affordance::AffConstPtr getAffordance() const  = 0; //should throw exception if doesn't apply
    virtual affordance::ManipulatorStateConstPtr getManipulator() const = 0; //should throw exception if doesn't apply
    virtual RelationStatePtr getRelationState() const = 0; //todo should throw exception if doesn't apply

    //mutators
public:
    virtual void setAffordance(affordance::AffConstPtr affordance) = 0;
    virtual void setManipulator(affordance::ManipulatorStateConstPtr manipulator) = 0;
    virtual void setRelationState(RelationStatePtr relationState) = 0;
    virtual drc::contact_goal_t toLCM() = 0;
};  //class AtomicConstraint

typedef boost::shared_ptr<AtomicConstraint> AtomicConstraintPtr;
typedef boost::shared_ptr<const AtomicConstraint> AtomicConstraintConstPtr;

} //namespace action_authoring


#endif //ATOMIC_CONSTRAINT_H
