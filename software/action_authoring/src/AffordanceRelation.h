#pragma once
#include "Affordance.h"

class AffordanceRelation{
 public:
    typedef enum {
      UNDEFINED,
      TANGENT,
      OFFSET,
      NORMAL
    } RelationType;

 protected:
  Affordance* m_affordance1;
  Affordance* m_affordance2;
  RelationType m_relationType;

 public:
  // Constructor
  AffordanceRelation(Affordance* affordance1, Affordance* affordance2, AffordanceRelation::RelationType relationType);

  //Accessors
  Affordance* getAffordance1() { return m_affordance1; };
  Affordance* getAffordance2() { return m_affordance2; };
  RelationType getRelationType() { return m_relationType; };
};
