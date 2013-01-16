#include "Constraint.h"

Constraint::Constraint(std::string name, Affordance* affordance1, Affordance* affordance2, Constraint::ConstraintType constraintType) {
	m_name = name;
	m_affordance1 = affordance1;
	m_affordance2 = affordance2;
	m_constraintType = constraintType;
}

void Constraint::setName(std::string name) {
	m_name = name;
}

void Constraint::setAffordance1(Affordance* affordance) {
	m_affordance1 = affordance;
}

void Constraint::setAffordance2(Affordance* affordance) {
        m_affordance2 = affordance;
}

Affordance* Constraint::getAffordance1() {
        return m_affordance1;
}

Affordance* Constraint::getAffordance2() {
        return m_affordance2;
}

Constraint::ConstraintType Constraint::getConstraintType() {
  return m_constraintType;
}

