#include "AffordanceRelation.h"

AffordanceRelation::AffordanceRelation(Affordance* affordance1, Affordance* affordance2, AffordanceRelation::RelationType relationType) {
  m_affordance1 = affordance1;
  m_affordance2 = affordance2;
  m_relationType = relationType;
};
