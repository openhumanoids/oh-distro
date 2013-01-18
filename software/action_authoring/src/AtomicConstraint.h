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
protected:
  std::string _name;
  /**affordances being related*/
  affordance::AffPtr _a1, _a2;
  AtomicConstraintType _constraintType;

  //--------constructor
 public:	  
	AtomicConstraint(std::string name,
					 affordance::AffPtr &affordance1, affordance::AffPtr &affordance2,
					 AtomicConstraintType type);

	// setters
	void setName(std::string name);
	void setAffordance1(affordance::AffPtr affordance1);
	void setAffordance2(affordance::AffPtr affordance2);

	//observers
	std::string getName();
	affordance::AffPtr getAffordance1();
	affordance::AffPtr getAffordance2();

}; //class action_authoring 

	typedef boost::shared_ptr<AtomicConstraint> AtomicConstraintPtr;

} //namespace action_authoring
