#ifndef AFFORDANCE_RELATION_H
#define AFFORDANCE_RELATION_H

#include "boost/shared_ptr.hpp"
#include "affordance/AffordanceState.h"

namespace action_authoring
{

/**todo: add comment*/
class AtomicConstraint
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
 private:
    affordance::AffPtr _affordance1;
    affordance::AffPtr _affordance2;
    RelationType _relationType;

    //------------Constructor--------
 public:
    AtomicConstraint(affordance::AffPtr affordance1, affordance::AffPtr affordance2, 
		       AtomicConstraint::RelationType relationType);

  //---------------Setters
  void setAffordance1(affordance::AffPtr a1) { _affordance1 = a1; };
  void setAffordance2(affordance::AffPtr a2) { _affordance2 = a2; };   

  //---------------Accessors
  affordance::AffPtr getAffordance1() { return _affordance1; };
  affordance::AffPtr getAffordance2() { return _affordance2; };
  RelationType getRelationType() { return _relationType; };
};  //class AtomicConstraint
 
 typedef boost::shared_ptr<AtomicConstraint> AtomicConstraintPtr;
 typedef boost::shared_ptr<const AtomicConstraint> AtomicConstraintConstPtr;

} //namespace action_authroing


#endif //AFFORDANCE_RELATION_H
