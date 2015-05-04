#include <stdexcept>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include "drake/RigidBodyManipulator.h"
#include "lcmtypes/drc/atlas_fall_detector_status_t.hpp"
#include "lcmtypes/drc/utime_t.hpp"
#include "lcmtypes/drc/foot_contact_estimate_t.hpp"
#include "lcmtypes/drc/controller_status_t.hpp"
#include "RobotStateDriver.hpp"

enum FootID {RIGHT, LEFT};

class AtlasFallDetector {
public:
  ~AtlasFallDetector() {}

  AtlasFallDetector(std::shared_ptr<RigidBodyManipulator> model);
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

private:
  std::shared_ptr<RigidBodyManipulator> model;
  std::shared_ptr<RobotStateDriver> state_driver;
  std::map<FootID, bool> foot_contact;
  std::map<FootID, int> foot_body_ids;
  DrakeRobotState robot_state;
  bool controller_is_active;
  double g = 9.81;
  double icp_exit_time = NAN;
  double icp_debounce_threshold = 0.01;

  Vector2d getICP();
  double getSupportFootHeight();
  Matrix3Xd getVirtualSupportPolygon ();
  bool isICPCaptured();
};