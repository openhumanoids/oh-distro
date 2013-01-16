#include "Action.h"
  
Action::Action(std::string name) {
	m_name = name;
	return;
}

void Action::addConstraint(Constraint* constraint) {
	m_constraints.push_back(constraint);
	return;
}

void Action::removeConstraint(int index) {
  m_constraints.erase(m_constraints.begin() + index);
  return;
}

void Action::reorderConstraint(int old_index, int new_index) {
  return;
}
