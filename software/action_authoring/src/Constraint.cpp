#include "Constraint.h"
#include <cstdlib>

using namespace action_authoring;
using namespace std;
Constraint::Constraint(const string &name, ConstraintType constraintType) 
  :  _constraintType(constraintType),
     _name(name),
     _affordanceRelation()
{
}

Constraint::Constraint(const string &name, AffRelationPtr affordanceRelation) 
{
  //todo : user constructor initialier list as in above constructor
  _constraintType = Constraint::ATOMIC;
  _name = name;
  _affordanceRelation = affordanceRelation;
}

void Constraint::addConstraint(ConstraintPtr constraint) {
  if (_constraintType == Constraint::ATOMIC)
    {
      throw InvalidMethodCallForContraintTypeException("Cannot add a constraint to an atomic constraint.");
    }

  _constraints.push_back(constraint);
}

void Constraint::getConstraints(vector<ConstraintPtr> &constraints) 
{
  if (_constraintType == Constraint::ATOMIC) 
    {
      throw InvalidMethodCallForContraintTypeException("Cannot get constraints for an atomic constraint.");
    }
  constraints.clear();
  constraints.insert(constraints.end(),
		     _constraints.begin(),
		     _constraints.end());
}

AffRelationPtr Constraint::getAffordanceRelation() {
  if (_constraintType != Constraint::ATOMIC) {
   throw InvalidMethodCallForContraintTypeException("Cannot get affordance relation for non-atomic constraint.");
  }
  return _affordanceRelation;
}


void Constraint::setAffordanceRelation(AffRelationPtr affordanceRelation) {
  if (_constraintType != Constraint::ATOMIC) {
   throw InvalidMethodCallForContraintTypeException("Cannot get affordance relation for non-atomic constraint.");
  }
  _affordanceRelation = affordanceRelation;
}
