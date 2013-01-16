#pragma once

#include <string>
#include "Affordance.h"

class Constraint {
 public:
  typedef enum {
    TANGENT,
    NORMAL} ConstraintType;

 private:
	std::string m_name;
	Affordance* m_affordance1;
	Affordance* m_affordance2;
	ConstraintType m_constraintType;
 public:	  
	Constraint(std::string name, Affordance* affordance1, Affordance* affordance2, ConstraintType constraintType);
	void setName(std::string name);
	void setAffordance1(Affordance* affordance);
	void setAffordance2(Affordance* affordance);
	std::string getName() { return m_name; }
	Affordance* getAffordance1();
	Affordance* getAffordance2();
	ConstraintType getConstraintType();
};
