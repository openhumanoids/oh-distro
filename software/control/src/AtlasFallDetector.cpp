#include "AtlasFallDetector.hpp"
#include "convexHull.hpp"
#include "lcmtypes/drc/recovery_trigger_t.hpp"

/* Monitors foot contacts and robot state, and applies heuristics to trigger the 
   recovery planner, and the bracing planner. */

AtlasFallDetector::AtlasFallDetector(std::shared_ptr<RigidBodyManipulator> model) {
  this->model = model;
  std::vector<std::string> state_coordinate_names;
  int num_states = model->num_positions + model->num_velocities;
  state_coordinate_names.reserve(num_states);
  this->robot_state.q.resize(num_states);
  this->robot_state.qd.resize(num_states);

  for (int i=0; i < num_states; ++i) {
    state_coordinate_names.push_back(model->getStateName(i));
  }

  this->state_driver.reset(new RobotStateDriver(state_coordinate_names));

  this->findFootIDS();

  this->foot_contact[RIGHT] = false;
  this->foot_contact[LEFT] = false;

  this->debounce.reset(new Debounce());
  this->debounce->t_low_to_high = 0.0;

  if (!this->lcm.good()) {
    throw std::runtime_error("LCM is not good");
  }
  this->lcm.subscribe("FOOT_CONTACT_ESTIMATE", &AtlasFallDetector::handleFootContact, this);
  this->lcm.subscribe("EST_ROBOT_STATE", &AtlasFallDetector::handleRobotState, this);
  this->lcm.subscribe("CONTROLLER_STATUS", &AtlasFallDetector::handleControllerStatus, this);
  this->lcm.subscribe("ATLAS_BEHAVIOR_COMMAND", &AtlasFallDetector::handleAtlasBehavior, this);

}

void AtlasFallDetector::findFootIDS() {
  for (int i=0; i < this->model->num_bodies; ++i) {
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

void AtlasFallDetector::handleAtlasBehavior(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const drc::atlas_behavior_command_t* msg) {
  if (msg->command != "user" || msg->command != "USER") {
    this->controller_is_active = false;
  }
}

void AtlasFallDetector::handleFootContact(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const drc::foot_contact_estimate_t* msg) {
  this->foot_contact[RIGHT] = msg->right_contact > 0.5;
  this->foot_contact[LEFT] = msg->left_contact > 0.5;
}

void AtlasFallDetector::handleRobotState(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const drc::robot_state_t* msg) {
  icp_cached = false;
  this->state_driver->decode(msg, &(this->robot_state));
  this->model->doKinematicsNew(robot_state.q, robot_state.qd);
  bool icp_is_ok = this->debounce->update(this->robot_state.t, this->isICPCaptured());
  bool icp_is_capturable = this->isICPCapturable();
  if (icp_is_capturable) icp_far_away_time = NAN;
  else if (!icp_is_capturable && isnan(icp_far_away_time)){
    icp_far_away_time = robot_state.t;
  } else {
    if (robot_state.t - icp_far_away_time < bracing_min_trigger_time)
      icp_is_capturable = true;
  }

  if (this->controller_is_active) {
    drc::atlas_fall_detector_status_t fall_msg;
    fall_msg.utime = static_cast<int64_t> (robot_state.t * 1e6);
    fall_msg.falling = !icp_is_ok;
    Vector2d icp = this->getICP();
    fall_msg.icp[0] = icp(0);
    fall_msg.icp[1] = icp(1);
    fall_msg.icp[2] = this->getSupportFootHeight();
    fall_msg.measured_cop[0] = NAN;
    fall_msg.measured_cop[1] = NAN;
    fall_msg.measured_cop[2] = NAN;
    this->lcm.publish("ATLAS_FALL_STATE", &fall_msg);

    if (~icp_is_capturable){
      // goto bracing
      drc::utime_t trigger_msg;
      trigger_msg.utime = static_cast<int64_t> (robot_state.t * 1e6);
      this->lcm.publish("BRACE_FOR_FALL", &trigger_msg);
    } else if (!icp_is_ok) {
      drc::recovery_trigger_t trigger_msg;
      trigger_msg.activate = true;
      trigger_msg.override = false;
      trigger_msg.utime = static_cast<int64_t> (robot_state.t * 1e6);
      this->lcm.publish("RECOVERY_TRIGGER", &trigger_msg);
    }
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
  if (icp_cached)
    return icp_cache;
  else {
    auto com = this->model->centerOfMass<double>(1);
    Vector3d com_pos = com.value();
    Vector3d com_vel = com.gradient().value() * this->robot_state.qd;
    Vector2d icp = com_pos.head<2>() + std::sqrt((com_pos(2) - this->getSupportFootHeight()) / (-this->model->a_grav(5))) * com_vel.head<2>();
    icp_cached = true;
    icp_cache = icp;
    return icp;
  }
}

double AtlasFallDetector::getSupportFootHeight() {
  std::map<FootID, double> sole_zs;
  for (std::map<FootID, int>::iterator foot = this->foot_body_ids.begin(); foot != foot_body_ids.end(); ++foot) {
    Matrix3Xd contact_pts;
    this->model->getTerrainContactPoints(*this->model->bodies[foot->second], contact_pts);
    Matrix3Xd contact_pts_in_world = this->model->forwardKinNew(contact_pts, foot->second, 0, 0, 0).value();
    // std::set<int> body_ids = {foot->second};
    // Matrix3Xd contact_pts_in_world(3, this->model->bodies[foot->second]->contact_pts.cols());
    // this->model->getContactPositions(contact_pts_in_world, body_ids);
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

Matrix3Xd AtlasFallDetector::getVirtualSupportPolygon () {
  Matrix3Xd all_contact_pts(3, 0);

  for (std::map<FootID, int>::iterator foot = this->foot_body_ids.begin(); foot != foot_body_ids.end(); ++foot) {
    // std::set<int> body_ids = {foot->second};
    // Matrix3Xd contact_pts_in_world(3, this->model->bodies[foot->second]->contact_pts.cols());
    // this->model->getContactPositions(contact_pts_in_world, body_ids);

    Matrix3Xd contact_pts;
    this->model->getTerrainContactPoints(*this->model->bodies[foot->second], contact_pts);
    Matrix3Xd contact_pts_in_world = this->model->forwardKinNew(contact_pts, foot->second, 0, 0, 0).value();
    if (!this->foot_contact.at(foot->first)) {
      // If foot is out of contact, only use its center to define the support polygon
      for (int i=0; i < 3; ++i) {
        contact_pts_in_world(i, 0) = contact_pts_in_world.row(i).mean();
      }
      contact_pts_in_world.conservativeResize(3, 1);
    }
    all_contact_pts.conservativeResize(3, all_contact_pts.cols() + contact_pts_in_world.cols());
    all_contact_pts.block(0, all_contact_pts.cols() - contact_pts_in_world.cols(), 3, contact_pts_in_world.cols()) = contact_pts_in_world;
  }
  return all_contact_pts;
}

bool AtlasFallDetector::isICPCaptured() {
  Vector2d icp = this->getICP();
  Matrix3Xd support_pts = this->getVirtualSupportPolygon();
  return in_convex_hull(support_pts.topRows(2), icp);
}

bool AtlasFallDetector::isICPCapturable() {
  Vector2d icp = this->getICP();
  Vector3d com; this->model->getCOM(com);
  double dist_from_com = (icp - com.block(0, 0, 2, 1)).norm();
  return dist_from_com < icp_capturable_radius;
}
