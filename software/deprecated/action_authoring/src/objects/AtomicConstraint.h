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
protected:
	double _timeLowerBound;
	double _timeUpperBound;

    //---------------Accessors
public:
    virtual affordance::AffConstPtr getAffordance() const  = 0; //should throw exception if doesn't apply
    virtual affordance::ManipulatorStateConstPtr getManipulator() const = 0; //should throw exception if doesn't apply
    virtual RelationStatePtr getRelationState() const = 0; //todo should throw exception if doesn't apply
    virtual double getTimeLowerBound() const = 0;
    virtual double getTimeUpperBound() const = 0;

    //mutators
public:
    virtual void setAffordance(const affordance::GlobalUID &affordanceUID) = 0;
    virtual void setManipulator(const affordance::GlobalUID &manipulatorUID) = 0;
    virtual void setRelationState(RelationStatePtr relationState) = 0;
    virtual void setTimeLowerBound(double timeLowerBound) = 0;
    virtual void setTimeUpperBound(double timeUpperBound) = 0;
    
public:
    virtual drc::contact_goal_t toLCM() = 0;
};  //class AtomicConstraint

typedef boost::shared_ptr<AtomicConstraint> AtomicConstraintPtr;
typedef boost::shared_ptr<const AtomicConstraint> AtomicConstraintConstPtr;

} //namespace action_authoring


#endif //ATOMIC_CONSTRAINT_H
