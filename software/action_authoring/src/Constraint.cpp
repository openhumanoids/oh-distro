#include "Constraint.h"
#include <cstdlib>

using namespace action_authoring;
using namespace std;
Constraint::Constraint(const string &name, const ConstraintType &constraintType) 
  :  m_constraintType(constraintType),
     m_name(name)
     m_affordanceRelation(NULL)
{
}

Constraint::Constraint(const string &name, const AffordanceRelation* affordanceRelation) 
{
  //todo : user constructor initialier list as in above constructor
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


