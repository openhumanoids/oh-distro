#include "AtomicConstraint.h"
using namespace action_authoring;
using namespace std;
using namespace affordance;

AtomicConstraint::AtomicConstraint(string name,
								   AffPtr &affordance1, AffPtr &affordance2,
								   AtomicConstraintType type)
 	 : _name(name), _a1(affordance1), _a2(affordance2), _constraintType(type)
{
}

string AtomicConstraint::getName()
{
	return _name;
}

void AtomicConstraint::setName(string name) 
{
    _name = name;
}

void AtomicConstraint::setAffordance1(affordance::AffPtr affordance1)
{
    _a1 = affordance1;
}

void AtomicConstraint::setAffordance2(affordance::AffPtr affordance2)
{
    _a2 = affordance2;
}

affordance::AffPtr AtomicConstraint::getAffordance1()
{
    return _a1;
}

affordance::AffPtr AtomicConstraint::getAffordance2()
{
    return _a2;
}
