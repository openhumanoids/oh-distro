#pragma once

#include "Constraint.h"
#include <vector>

class Action {

private:
  std::vector<Constraint*> m_constraints;
  std::string m_name;
  
public:
  Action(std::string name);
  void addConstraint(Constraint* constraint);
  void reorderConstraint(int old_index, int new_index);
  void removeConstraint(int index);
  std::string getName() { return m_name; };
  std::vector<Constraint*> getConstraints() { return m_constraints; };
};
