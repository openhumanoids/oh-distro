#include "AtlasFallDetector.hpp"
#include "drake/util/convexHull.h"
#include "lcmtypes/drc/recovery_trigger_t.hpp"

using namespace Eigen;

/* Monitors foot contacts and robot state, and applies heuristics to trigger the 
   recovery planner, and the bracing planner. */

AtlasFallDetector::AtlasFallDetector(std::shared_ptr<RigidBodyTree> model, bool sim_override) :
    kinematics_cache(model->bodies)
{
  // in sim atlas_status is never published, so make sure the fall detector can still function
  this->atlas_is_in_user = sim_override;

  this->model = model;
  std::vector<std::string> state_coordinate_names;
  int num_states = model->num_positions + model->num_velocities;
  state_coordinate_names.reserve(num_states);
  this->robot_state.q.resize(model->num_positions);
  this->robot_state.qd.resize(model->num_velocities);

  for (int i=0; i < num_states; ++i) {
    state_coordinate_names.push_back(model->getStateName(i));
  }

  this->state_driver.reset(new RobotStateDriver(state_coordinate_names));

  this->findFootIDS();

  this->icp_is_ok_debounce.reset(new Debounce());
  this->icp_is_ok_debounce->t_low_to_high = 0.0;
  this->icp_is_capturable_debounce.reset(new Debounce());
  this->icp_is_capturable_debounce->t_low_to_high = 0.0;
  this->icp_is_capturable_debounce->t_high_to_low = this->bracing_min_trigger_time;

  this->resetState();

  if (!this->lcm.good()) {
    throw std::runtime_error("LCM is not good");
  }
  this->lcm.subscribe("FOOT_CONTACT_ESTIMATE", &AtlasFallDetector::handleFootContact, this);
  this->lcm.subscribe("EST_ROBOT_STATE", &AtlasFallDetector::handleRobotState, this);
  this->lcm.subscribe("CONTROLLER_STATUS", &AtlasFallDetector::handleControllerStatus, this);
  this->lcm.subscribe("ATLAS_STATUS", &AtlasFallDetector::handleAtlasStatus, this);
}

void AtlasFallDetector::resetState() {
  this->foot_contact[RIGHT] = false;
  this->foot_contact[LEFT] = false;
  this->foot_contact_valid = false;
  this->bracing_latch = false;
  this->icp_is_ok_debounce->reset(true);
  this->icp_is_capturable_debounce->reset(true);
}

void AtlasFallDetector::findFootIDS() {
  for (int i=0; i < this->model->bodies.size(); ++i) {
    if (this->model->bodies[i]->linkname == "r_foot") {
      this->foot_body_ids[RIGHT] = i;
    } else if (this->model->bodies[i]->linkname == "l_foot") {
      this->foot_body_ids[LEFT] = i;
    }
  }

  if (this->foot_body_ids.find(RIGHT) == this->foot_body_ids.end()) {
    throw std::runtime_error("could not find r_foot body");
  }
  if (this->foot_body_ids.find(LEFT) == this->foot_body_ids.end()) {
    throw std::runtime_error("could not find l_foot body");
  }
}

void AtlasFallDetector::handleAtlasStatus(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const atlas::status_t* msg) {
  this->atlas_is_in_user = msg->behavior == msg->BEHAVIOR_USER;
}

void AtlasFallDetector::handleFootContact(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const drc::foot_contact_estimate_t* msg) {
  this->foot_contact[RIGHT] = msg->right_contact > 0.5;
  this->foot_contact[LEFT] = msg->left_contact > 0.5;
  this->foot_contact_valid = true
;}

void AtlasFallDetector::handleRobotState(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const bot_core::robot_state_t* msg) {
  // std::cout << "right: " << this->foot_contact.at(RIGHT) << " left: " << this->foot_contact.at(LEFT);

  // std::cout << " active: " << controller_is_active << " user: " << atlas_is_in_user << " contact valid: " << foot_contact_valid;
  if (this->controller_is_active && this->atlas_is_in_user && this->foot_contact_valid) {
    this->state_driver->decode(msg, &(this->robot_state));
    kinematics_cache.initialize(robot_state.q, robot_state.qd);
    this->model->doKinematics(kinematics_cache, true);

    Vector2d icp = this->getICP();
    bool icp_is_ok = this->icp_is_ok_debounce->update(this->robot_state.t, this->isICPCaptured(icp));
    bool icp_is_capturable = this->icp_is_capturable_debounce->update(this->robot_state.t, this->isICPCapturable(icp));
    // std::cout << " icp ok: " << icp_is_ok << " icp capturable: " << icp_is_capturable << std::endl;
    if (!icp_is_capturable && !this->bracing_latch){
      std::cout << "bracing!" << std::endl;
      this->bracing_latch = true;
    }

    drc::fall_detector_status_t fall_msg;
    fall_msg.utime = static_cast<int64_t> (robot_state.t * 1e6);
    fall_msg.falling = !icp_is_ok;
    fall_msg.bracing = bracing_latch;
    fall_msg.icp[0] = icp(0);
    fall_msg.icp[1] = icp(1);
    fall_msg.icp[2] = this->getSupportFootHeight();
    fall_msg.measured_cop[0] = NAN;
    fall_msg.measured_cop[1] = NAN;
    fall_msg.measured_cop[2] = NAN;
    this->lcm.publish("FALL_STATE", &fall_msg);

    if (this->bracing_latch){
      // goto bracing
      drc::recovery_trigger_t trigger_msg;
      trigger_msg.activate = true;
      trigger_msg.override = false;
      trigger_msg.utime = static_cast<int64_t> (robot_state.t * 1e6);
      this->lcm.publish("BRACE_FOR_FALL", &trigger_msg);
    } else if (!icp_is_ok) {
      drc::recovery_trigger_t trigger_msg;
      trigger_msg.activate = true;
      trigger_msg.override = false;
      trigger_msg.utime = static_cast<int64_t> (robot_state.t * 1e6);
      this->lcm.publish("RECOVERY_TRIGGER", &trigger_msg);
    }
  } else {
    this->resetState();
    // std::cout << std::endl;
  }

}

void AtlasFallDetector::handleControllerStatus(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const drc::controller_status_t* msg) {
  this->controller_is_active = (msg->state == msg->STANDING ||
                                msg->state == msg->WALKING || 
                                msg->state == msg->MANIPULATING || 
                                msg->state == msg->RECOVERING);
}

Vector2d AtlasFallDetector::getICP() {
  Vector3d com_pos = this->model->centerOfMass(kinematics_cache);
  Vector3d com_vel = this->model->centerOfMassJacobian(kinematics_cache) * this->robot_state.qd;
  Vector2d icp = com_pos.head<2>() + std::sqrt((com_pos(2) - this->getSupportFootHeight()) / (-this->model->a_grav(5))) * com_vel.head<2>();
  return icp;
}

double AtlasFallDetector::getSupportFootHeight() {
  std::map<FootID, double> sole_zs;
  for (std::map<FootID, int>::iterator foot = this->foot_body_ids.begin(); foot != foot_body_ids.end(); ++foot) {
    Matrix3Xd contact_pts;
    this->model->getTerrainContactPoints(*this->model->bodies[foot->second], contact_pts);
    Matrix3Xd contact_pts_in_world = this->model->transformPoints(kinematics_cache, contact_pts, foot->second, 0);
    sole_zs[foot->first] = contact_pts_in_world.row(2).mean();
  }
  if ((this->foot_contact.at(RIGHT) && this->foot_contact.at(LEFT)) || (!this->foot_contact.at(RIGHT) && !this->foot_contact.at(LEFT))) {
    return std::min(sole_zs.at(RIGHT), sole_zs.at(LEFT));
  } else if (this->foot_contact.at(RIGHT) && !this->foot_contact.at(LEFT)) {
    return sole_zs.at(RIGHT);
  } else if (this->foot_contact.at(LEFT) && !this->foot_contact.at(RIGHT)) {
    return sole_zs.at(LEFT);
  } else {
    throw std::runtime_error("should never get here"); 
  }
}

Matrix3Xd AtlasFallDetector::getVirtualSupportPolygon (bool shrink_noncontact_foot) {
  Matrix3Xd all_contact_pts(3, 0);

  for (std::map<FootID, int>::iterator foot = this->foot_body_ids.begin(); foot != foot_body_ids.end(); ++foot) {
    Matrix3Xd contact_pts;
    this->model->getTerrainContactPoints(*this->model->bodies[foot->second], contact_pts);
    Matrix3Xd contact_pts_in_world = this->model->transformPoints(kinematics_cache, contact_pts, foot->second, 0);
    if (shrink_noncontact_foot) {
      if (!this->foot_contact.at(foot->first)) {
        // If foot is out of contact, only use its center to define the support polygon
        for (int i=0; i < 3; ++i) {
          contact_pts_in_world(i, 0) = contact_pts_in_world.row(i).mean();
        }
        contact_pts_in_world.conservativeResize(3, 1);
      }
    }
    all_contact_pts.conservativeResize(3, all_contact_pts.cols() + contact_pts_in_world.cols());
    all_contact_pts.block(0, all_contact_pts.cols() - contact_pts_in_world.cols(), 3, contact_pts_in_world.cols()) = contact_pts_in_world;
  }
  return all_contact_pts;
}

bool AtlasFallDetector::isICPCaptured(Vector2d icp) {
  Matrix3Xd support_pts = this->getVirtualSupportPolygon();
  return inConvexHull(support_pts.topRows<2>(), icp);
}

bool AtlasFallDetector::isICPCapturable(Vector2d icp) {
  return signedDistanceInsideConvexHull(this->getVirtualSupportPolygon(false), icp) > -icp_capturable_radius;
}
