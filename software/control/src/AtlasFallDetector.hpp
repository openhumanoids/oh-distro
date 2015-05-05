#include <stdexcept>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include "drake/RigidBodyManipulator.h"
#include "lcmtypes/drc/atlas_fall_detector_status_t.hpp"
#include "lcmtypes/drc/utime_t.hpp"
#include "lcmtypes/drc/foot_contact_estimate_t.hpp"
#include "lcmtypes/drc/controller_status_t.hpp"
#include "lcmtypes/drc/atlas_behavior_command_t.hpp"
#include "RobotStateDriver.hpp"

enum FootID {RIGHT, LEFT};

enum DebounceState {LOW, GOING_HIGH, HIGH, GOING_LOW};


class Debounce {
public:
  ~Debounce() {}

  double t_low_to_high = 0.01;
  double t_high_to_low = 0.01;

  bool update(double t, bool input) {
    if (input) {
      switch (this->state) {
        case LOW:
          this->state = GOING_HIGH;
          this->t_transition_start = t;
          break;
        case GOING_LOW:
          this->state = HIGH;
          break;
        }
    } else {
      switch (this->state) {
        case HIGH:
          this->state = GOING_LOW;
          this->t_transition_start = t;
          break;
        case GOING_HIGH:
          this->state = LOW;
          break;

      }
    }

    if (this->state == GOING_HIGH && t - this->t_transition_start >= this->t_low_to_high) {
      this->state = HIGH;
    } else if (this->state == GOING_LOW && t - this->t_transition_start >= this->t_high_to_low) {
      this->state = LOW;
    }

    switch (this->state) {
      case LOW:
        return false;
      case GOING_HIGH:
        return false;
      case HIGH:
        return true;
      case GOING_LOW:
        return true;
      default:
        throw std::runtime_error("should never get here");
    }
  }

private:
  DebounceState state = LOW;
  double t_transition_start = 0;
};


class AtlasFallDetector {
public:
  ~AtlasFallDetector() {}

  AtlasFallDetector(std::shared_ptr<RigidBodyManipulator> model);
  void run() {
    while(0 == this->lcm.handle());
  }

private:
  std::shared_ptr<RigidBodyManipulator> model;
  std::shared_ptr<RobotStateDriver> state_driver;
  std::unique_ptr<Debounce> debounce;
  std::map<FootID, bool> foot_contact;
  std::map<FootID, int> foot_body_ids;
  DrakeRobotState robot_state;
  bool controller_is_active;
  double g = 9.81;
  double icp_exit_time = NAN;
  double icp_debounce_threshold = 0.01;
  lcm::LCM lcm;

  Vector2d getICP();
  double getSupportFootHeight();
  Matrix3Xd getVirtualSupportPolygon ();
  bool isICPCaptured();

  void findFootIDS();
  void handleFootContact(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const drc::foot_contact_estimate_t* msg);
  void handleRobotState(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const drc::robot_state_t* msg);
  void handleControllerStatus(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const drc::controller_status_t* msg);
  void handleAtlasBehavior(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const drc::atlas_behavior_command_t* msg);
};