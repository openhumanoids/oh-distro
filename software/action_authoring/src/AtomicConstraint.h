#pragma once

#include <string>
#include "affordance/AffordanceState.h"
#include <boost/shared_ptr.hpp>

namespace action_authoring
{

/**Represents an immutable constraint between two affordances*/
class AtomicConstraint
{
public:

/**type of constraint between two affordances*/
  enum AtomicConstraintType
  {
    TANGENT,
    NORMAL,
    RELATIVE_POSE
  };


  //--------fields
public:
  const std::string _name;

  /**affordances being related*/
  const affordance::AffPtr _a1, _a2;

  const AtomicConstraintType _constraintType;

  //--------constructor
 public:	  
	AtomicConstraint(const std::string &name,
					 const affordance::AffPtr affordance1, const affordance::AffPtr &affordance2,
					 const AtomicConstraintType &type);

	//observers
	std::string getName() const;

}; //class action_authoring

	typedef boost::shared_ptr<AtomicConstraint> AtomicConstraintPtr;

} //namespace action_authoring
