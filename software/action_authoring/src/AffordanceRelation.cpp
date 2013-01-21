#include "AffordanceRelation.h"

using namespace action_authoring;
using namespace affordance;

/**todo: Comment here
   @param affordance1
   @param affordance2
   @param relationType*/
AffordanceRelation::AffordanceRelation(AffPtr affordance1, 
				       AffPtr affordance2, 
				       AffordanceRelation::RelationType relationType)
  : _affordance1(affordance1), 
    _affordance2(affordance2), _relationType(relationType)
{

}
