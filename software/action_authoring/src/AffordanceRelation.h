#ifndef AFFORDANCE_RELATION_H
#define AFFORDANCE_RELATION_H

#include "boost/shared_ptr.hpp"
#include "affordance/AffordanceState.h"

namespace action_authoring
{

/**todo: add comment*/
class AffordanceRelation
{
  
  //----------Enumerations
 public:
    typedef enum {
      UNDEFINED,
      TANGENT,
      OFFSET, //todo: small comment here
      NORMAL
    } RelationType;


    //------------fields
 protected:
    const affordance::AffConstPtr _affordance1;
    const affordance::AffConstPtr _affordance2;
    const RelationType _relationType;

    //------------Constructor--------
 public:
    AffordanceRelation(affordance::AffConstPtr affordance1, affordance::AffConstPtr affordance2, 
		     const AffordanceRelation::RelationType &relationType);

  //---------------Accessors
  affordance::AffConstPtr getAffordance1() const { return _affordance1; };
  affordance::AffConstPtr getAffordance2() const { return _affordance2; };
  RelationType getRelationType() const { return _relationType; };
};  //class AffordanceRelation
 
 typedef boost::shared_ptr<AffordanceRelation> AffRelationPtr;
 typedef boost::shared_ptr<const AffordanceRelation> AffRelationConstPtr;

} //namespace action_authroing


#endif //AFFORDANCE_RELATION_H
