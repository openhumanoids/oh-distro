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
    _timeUpperBound = 0.0;
    _timeLowerBound = 0.0;
}   

ConstraintMacro::ConstraintMacro(const string &name, AtomicConstraintPtr atomicConstraint) :
    _name(name),
    _constraints(),
    _constraintType(ConstraintMacro::ATOMIC),
    _atomicConstraint(atomicConstraint)
{
    _timeLowerBound = atomicConstraint->getTimeLowerBound();
    _timeUpperBound = atomicConstraint->getTimeUpperBound();
}

void ConstraintMacro::appendConstraintMacro(ConstraintMacroPtr constraint)
{
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

AtomicConstraintPtr ConstraintMacro::getAtomicConstraint() const
{
    if (_constraintType != ConstraintMacro::ATOMIC)
    {
        throw InvalidMethodCallForContraintTypeException("Cannot get affordance relation for non-atomic constraint.");
    }

    return _atomicConstraint;
}

std::vector<drc::contact_goal_t> ConstraintMacro::toLCM()
{
    printf("toLCM called\n");
    std::vector<drc::contact_goal_t> lcmMessages;

    if (_constraintType == ConstraintMacro::ATOMIC)
    {
        printf("types is ATOMIC, calling toLCM on atomic constraint\n");
        lcmMessages.push_back(_atomicConstraint->toLCM());
        printf("ATOMIC toLCM complete\n");
    }
    else if (_constraintType == ConstraintMacro::SEQUENTIAL)
    {
        printf("type is SEQEUENTIAL\n");
        std::vector<drc::contact_goal_t> child_constraints;

        for (int i = 0; i < (int) _constraints.size(); i++)
        {
            printf("calling toLCM on child %i \n", i);
            child_constraints = _constraints[i]->toLCM();
            printf("child %i toLCM is finished\n", i);

            for (int j = 0; j < (int)child_constraints.size(); j++)
            {
                lcmMessages.push_back(child_constraints[j]);
            }
        }
    }
    else
    {
        printf("Don't know how to convert ConstraintMacro type to LCM. Ignoring.\n");
    }

    return lcmMessages;
}

