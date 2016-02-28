#include <stdexcept>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include "drake/systems/plants/RigidBodyTree.h"
#include "lcmtypes/drc/fall_detector_status_t.hpp"
#include "lcmtypes/bot_core/utime_t.hpp"
#include "lcmtypes/drc/foot_contact_estimate_t.hpp"
#include "lcmtypes/drc/controller_status_t.hpp"
#include "lcmtypes/drc/behavior_t.hpp"
#include "drake/lcmt_qp_controller_input.hpp"
#include "lcmtypes/atlas/status_t.hpp"
#include "RobotStateDriver.hpp"

enum FootID {RIGHT, LEFT};

enum DebounceState {LOW, GOING_HIGH, HIGH, GOING_LOW};


class Debounce {
public:
  ~Debounce() {}

  double t_low_to_high = 0.01;
  double t_high_to_low = 0.01;

  void reset(bool state) {
    if (state) {
      this->state = HIGH;
    } else {
      this->state = LOW;
    }
  }


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

  AtlasFallDetector(std::shared_ptr<RigidBodyTree> model, bool sim_override = false);
  void run() {
    while(0 == this->lcm.handle());
  }

private:
  std::shared_ptr<RigidBodyTree> model;
  std::shared_ptr<RobotStateDriver> state_driver;
  KinematicsCache<double> kinematics_cache;
  std::unique_ptr<Debounce> icp_is_ok_debounce;
  std::unique_ptr<Debounce> icp_is_capturable_debounce;
  std::map<FootID, int> foot_body_ids;

  bool foot_contact_valid = false;
  std::map<FootID, bool> foot_contact;
  DrakeRobotState robot_state;

  // Double lock to keep the fall detector from running unless
  // it is true that:
  //  -- the controller is in a controlled state (recovery, manip, walking, etc)
  //  -- the robot is in user
  bool controller_is_active = false;
  bool atlas_is_in_user = false;

  lcm::LCM lcm;

  double icp_capturable_radius = 0.10;
  double bracing_min_trigger_time = 0.4;
  bool bracing_latch = false;

  Eigen::Vector2d getICP();
  double getSupportFootHeight();
  Eigen::Matrix3Xd getVirtualSupportPolygon (bool shrink_noncontact_foot=true);
  bool isICPCaptured(Eigen::Vector2d icp);
  bool isICPCapturable(Eigen::Vector2d icp);

  void findFootIDS();
  void resetState();
  void handleFootContact(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const drc::foot_contact_estimate_t* msg);
  void handleRobotState(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const bot_core::robot_state_t* msg);
  void handleControllerStatus(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const drc::controller_status_t* msg);
  void handleAtlasStatus(const lcm::ReceiveBuffer* rbuf,
                         const std::string& chan,
                         const atlas::status_t* msg);
};
