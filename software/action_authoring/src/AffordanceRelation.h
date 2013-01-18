#pragma once
#include "affordance/AffordanceState.h"

/**todo: add comment*/
class AffordanceRelation{
  
  //----------Enumerations
 public:
    typedef enum {
      UNDEFINED,
      TANGENT,
      OFFSET,
      NORMAL
    } RelationType;


    //------------fields
 protected:
    const AffConstPtr m_affordance1;
    const AffConstPtr m_affordance2;
    const RelationType m_relationType;

    //------------Constructor--------
 public:
  AffordanceRelation(AffConstPtr affordance1, AffConstPtr affordance2, 
		     const AffordanceRelation::RelationType &relationType);

  //---------------Accessors
  AffConstPtr getAffordance1() const { return m_affordance1; };
  AffConstPtr getAffordance2() const { return m_affordance2; };
  RelationType getRelationType() const { return m_relationType; };
};
