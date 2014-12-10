#ifndef MANIPULATION_RELATION_H
#define MANIPULATION_RELATION_H

#include <boost/shared_ptr.hpp>
#include <affordance/AffordanceState.h>
#include <affordance/ManipulatorState.h>
#include "AffordanceManipMap.h"
#include "RelationState.h"
#include "AtomicConstraint.h"

namespace action_authoring
{
  
class ManipulationConstraint : public AtomicConstraint
{

  //----------Constructor
public:
  ManipulationConstraint(affordance::GlobalUID affordanceUID,
                         affordance::GlobalUID manipulatorUID,
                         AffordanceManipMap *amMap,
                         RelationStatePtr relationState);
    
  ManipulationConstraint(affordance::GlobalUID affordanceUID,
                         affordance::GlobalUID manipulatorUID,
                         AffordanceManipMap *amMap,
                         RelationStatePtr relationState,
                         double timeLowerBound,
                         double timeUpperBound);

    //----------Accessors
    affordance::AffConstPtr getAffordance() const
    {
      return _amMap->getAffordance(*_affordanceUID);
    }
    affordance::ManipulatorStateConstPtr getManipulator() const
    {
      return _amMap->getManipulator(*_manipulatorUID);
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

    //----mutators
    void setAffordance(const affordance::GlobalUID &affordanceUID);
    void setManipulator(const affordance::GlobalUID &manipulatorUID);

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
    boost::shared_ptr<affordance::GlobalUID> _affordanceUID;
    boost::shared_ptr<affordance::GlobalUID> _manipulatorUID;
    AffordanceManipMap *_amMap;
    
    RelationStatePtr _relationState;

}; // class ManipulationConstraint
typedef boost::shared_ptr<ManipulationConstraint> ManRelPtr;
typedef boost::shared_ptr<const ManipulationConstraint> ManRelConstPtr;

} //namespace action_authoring

#endif //MANIPULATION_RELATION_H
