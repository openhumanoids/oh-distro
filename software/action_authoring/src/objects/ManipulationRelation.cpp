#include "ManipulationRelation.h"

using namespace action_authoring;
using namespace affordance;

ManipulationRelation::
ManipulationRelation(AffConstPtr affordance,
		     ManipulatorStateConstPtr manipulator, 
		     RelationStatePtr relationState) 
  : _affordance(affordance), 
    _manipulator(manipulator),
    _relationState(relationState)
{
   
}
