#include "Constraint.h"
#include <cstdlib>

using namespace action_authoring;

Constraint::Constraint(char* name, ConstraintType constraintType) {
  m_constraintType = constraintType;
  m_name = name;
  m_affordanceRelation = NULL;
}

Constraint::Constraint(char* name, AffordanceRelation* affordanceRelation) {
  m_constraintType = Constraint::ATOMIC;
  m_name = name;
  m_affordanceRelation = affordanceRelation;
}

void Constraint::addConstraint(Constraint* constraint) {
  if (m_constraintType == Constraint::ATOMIC) {
	   throw InvalidMethodCallForContraintTypeException("Cannot add a constraint to an atomic constraint.");
  }
  m_constraints.push_back(constraint);
}

std::vector<Constraint*> Constraint::getConstraints() {
  if (m_constraintType == Constraint::ATOMIC) {
   throw InvalidMethodCallForContraintTypeException("Cannot get constraints for an atomic constraint.");
  }
  return m_constraints;
}

AffordanceRelation* Constraint::getAffordanceRelation() {
  if (m_constraintType != Constraint::ATOMIC) {
   throw InvalidMethodCallForContraintTypeException("Cannot get affordance relation for non-atomic constraint.");
  }
  return m_affordanceRelation;
}


