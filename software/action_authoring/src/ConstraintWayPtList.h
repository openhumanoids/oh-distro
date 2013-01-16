#pragma once

#include "AtomicConstraint.h"
#include <vector>
#include <boost/shared_ptr.hpp>

namespace action_authoring
{

/**sequence of atomic constriants*/
class ConstraintWayPtList
{
	//-------fields
public:
	const std::string _name;

private:
  std::vector<AtomicConstraintPtr> _orderedConstraints;
  
  //----constructor
public:
  ConstraintWayPtList(const std::string &name);

  //------mutators
  void addConstraint(const AtomicConstraintPtr &constraint);
  void swapConstraints(const int &index1, const int &index2);
  void removeConstraint(const int &index);

  //-------accessors
  void getConstraints(std::vector<AtomicConstraintPtr> constraints) const;
};

} //namespace action_authoring
