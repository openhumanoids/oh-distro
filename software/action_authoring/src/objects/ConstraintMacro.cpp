#include "ConstraintMacro.h"
#include <cstdlib>

using namespace action_authoring;
using namespace std;
ConstraintMacro::ConstraintMacro(const string &name, const ConstraintMacroType &constraintType) 
  :
   _name(name),
   _constraints(),
   _constraintType(constraintType),
   _atomicConstraint()
{
    _timeUpperBound = 2.0;
    _timeLowerBound = 0;
}

ConstraintMacro::ConstraintMacro(const string &name, AtomicConstraintConstPtr atomicConstraint) : 
  _name(name),
  _constraints(),
  _constraintType(ConstraintMacro::ATOMIC),
  _atomicConstraint(atomicConstraint)
{

}

void ConstraintMacro::appendConstraintMacro(ConstraintMacroPtr constraint) {
  if (_constraintType == ConstraintMacro::ATOMIC)
    {
      throw InvalidMethodCallForContraintTypeException("Cannot add a constraint to an atomic constraint.");
    }

  _constraints.push_back(constraint);
}

void ConstraintMacro::getConstraintMacros(vector<ConstraintMacroPtr> &constraints) const
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

AtomicConstraintConstPtr ConstraintMacro::getAtomicConstraint() const {
  if (_constraintType != ConstraintMacro::ATOMIC) {
   throw InvalidMethodCallForContraintTypeException("Cannot get affordance relation for non-atomic constraint.");
  }
  return _atomicConstraint;
}

