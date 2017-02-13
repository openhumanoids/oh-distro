//
// Created by manuelli on 2/13/17.
//

#ifndef CONTROL_CONTACT_PLAN_H
#define CONTROL_CONTACT_PLAN_H

#include "drake/systems/controllers/controlUtil.h"

namespace plan_eval{
class ContactPlan {
public:
  int next_contact_state_idx_;
  double last_contact_event_time_ = -INFINITY;
  std::vector<std::pair<ContactState, double>> contact_states_;

  ContactPlan();
  void addContactState(ContactState contact_state, double time);
  bool hasNextContactState();
  ContactState getCurrentContactState();
  ContactState getNextContactState();
  std::pair<ContactState, double> getNextContactStatePair();
  double getNextContactStateTime();
  void incrementCounter();
  // contact time should be in plan time
  void recordContactEvent(const double& contact_time);
  void clear();
  void printDebugInfo() const;
};
}

#endif //CONTROL_CONTACT_PLAN_H
