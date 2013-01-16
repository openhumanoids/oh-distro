#include "AtomicConstraint.h"
using namespace action_authoring;
using namespace std;
using namespace affordance;

AtomicConstraint::AtomicConstraint(const string &name,
								   const AffPtr affordance1, const AffPtr &affordance2,
								   const AtomicConstraintType &type)
 	 : _name(name), _a1(affordance1), _a2(affordance2), _constraintType(type)
{
}

string AtomicConstraint::getName() const
{
	return _name;
}
