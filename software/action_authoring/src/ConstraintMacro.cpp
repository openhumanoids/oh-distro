#include "ConstraintMacro.h"
#include <cstdlib>

using namespace action_authoring;
using namespace std;
ConstraintMacro::ConstraintMacro(const string &name, ConstraintMacroType constraintType) 
  :  _constraintType(constraintType),
     _name(name),
     _atomicConstraint()
{
}

ConstraintMacro::ConstraintMacro(const string &name, AtomicConstraintPtr atomicConstraint) 
{
  //todo : user constructor initialier list as in above constructor
  _constraintType = ConstraintMacro::ATOMIC;
  _name = name;
  _atomicConstraint = atomicConstraint;
}

void ConstraintMacro::addConstraintMacro(ConstraintMacroPtr constraint) {
  if (_constraintType == ConstraintMacro::ATOMIC)
    {
      throw InvalidMethodCallForContraintTypeException("Cannot add a constraint to an atomic constraint.");
    }

  _constraints.push_back(constraint);
}

void ConstraintMacro::getConstraintMacros(vector<ConstraintMacroPtr> &constraints) 
{
  if (_constraintType == ConstraintMacro::ATOMIC) 
    {
      throw InvalidMethodCallForContraintTypeException("Cannot get constraints for an atomic constraint.");
    }
  constraints.clear();
  constraints.insert(constraints.end(),
		     _constraints.begin(),
		     _constraints.end());
}

AtomicConstraintPtr ConstraintMacro::getAtomicConstraint() {
  if (_constraintType != ConstraintMacro::ATOMIC) {
   throw InvalidMethodCallForContraintTypeException("Cannot get affordance relation for non-atomic constraint.");
  }
  return _atomicConstraint;
}


void ConstraintMacro::setAtomicConstraint(AtomicConstraintPtr atomicConstraint) {
  if (_constraintType != ConstraintMacro::ATOMIC) {
   throw InvalidMethodCallForContraintTypeException("Cannot get affordance relation for non-atomic constraint.");
  }
  _atomicConstraint = atomicConstraint;
}
