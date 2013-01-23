#ifndef ATOMIC_CONSTRAINT_H
#define ATOMIC_CONSTRAINT_H

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
    affordance::AffConstPtr _affordance1;
    affordance::AffConstPtr _affordance2;
    RelationType _relationType;

    //------------Constructor--------
 public:
    AtomicConstraint(affordance::AffConstPtr affordance1, affordance::AffConstPtr affordance2, 
		     const AtomicConstraint::RelationType &relationType);

  //---------------Accessors
  affordance::AffConstPtr getAffordance1() const { return _affordance1; };
  affordance::AffConstPtr getAffordance2() const { return _affordance2; };
  RelationType getRelationType() const { return _relationType; };

  //mutators
  void setAffordance1(affordance::AffConstPtr affordance) {_affordance1 = affordance;}
  void setAffordance2(affordance::AffConstPtr affordance) {_affordance2 = affordance;}
  
};  //class AtomicConstraint
 
 typedef boost::shared_ptr<AtomicConstraint> AtomicConstraintPtr;
 typedef boost::shared_ptr<const AtomicConstraint> AtomicConstraintConstPtr;

} //namespace action_authroing


#endif //ATOMIC_CONSTRAINT_H
