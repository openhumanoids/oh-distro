//
// Created by manuelli on 2/13/17.
//
#include "contact_plan.h"
namespace plan_eval{

ContactPlan::ContactPlan() {
  next_contact_state_idx_ = 0;
}

void ContactPlan::addContactState(ContactState contact_state, double time) {
  contact_states_.push_back(std::make_pair(contact_state, time));
}

bool ContactPlan::hasNextContactState() {
  return (next_contact_state_idx_ < contact_states_.size());
}

ContactState ContactPlan::getCurrentContactState() {
  return this->contact_states_.at(next_contact_state_idx_ - 1).first;
}

ContactState ContactPlan::getNextContactState() {
  return this->contact_states_.at(next_contact_state_idx_).first;
}

std::pair<ContactState, double> ContactPlan::getNextContactStatePair() {
  return this->contact_states_.at(next_contact_state_idx_);
}

double ContactPlan::getNextContactStateTime() {
  return this->contact_states_.at(next_contact_state_idx_).second;
}

void ContactPlan::incrementCounter() {
  next_contact_state_idx_++;
}

void ContactPlan::clear() {
  this->contact_states_.clear();
  this->next_contact_state_idx_ = 0;
  this->last_contact_event_time_ = -INFINITY;
}

void ContactPlan::recordContactEvent(const double& contact_time) {
  this->last_contact_event_time_ = contact_time;
  this->incrementCounter();
}

void ContactPlan::printDebugInfo() const {
  std::cout << "------Contact Plan Debug Info------------\n";
  std::cout << "contact_states_.size() = " << contact_states_.size() << std::endl;
  std::cout << "next_contact_state_idx_ = " << next_contact_state_idx_ << std::endl;
  std::cout << "last_contact_event_time_ = " << last_contact_event_time_ << std::endl;
  std::cout << "----------------- \n ";
}

}