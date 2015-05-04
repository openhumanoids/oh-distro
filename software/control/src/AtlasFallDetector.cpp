#include "AtlasFallDetector.hpp"
#include "convexHull.hpp"

AtlasFallDetector::AtlasFallDetector(std::shared_ptr<RigidBodyManipulator> model) {
  this->model = model;
  std::vector<std::string> state_coordinate_names;
  int num_states = model->num_positions + model->num_velocities;
  state_coordinate_names.reserve(num_states);

  for (int i=0; i < num_states; ++i) {
    state_coordinate_names.push_back(model->getStateName(i));
    // std::cout << state_coordinate_names[i] << std::endl;
  }

  this->state_driver.reset(new RobotStateDriver(state_coordinate_names));

  this->findFootIDS();
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

void AtlasFallDetector::handleFootContact(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const drc::foot_contact_estimate_t* msg) {
  this->foot_contact[RIGHT] = msg->right_contact > 0.5;
  this->foot_contact[LEFT] = msg->left_contact > 0.5;
}

void AtlasFallDetector::handleRobotState(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const drc::robot_state_t* msg) {
  this->state_driver->decode(msg, &(this->robot_state));
  this->model->doKinematics(robot_state.q);

  drc::atlas_fall_detector_status_t msg;
  if (this->isICPCaptured()) {
  }

}

void AtlasFallDetector::handleControllerStatus(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const drc::controller_status_t* msg) {
  this->controller_is_active = (msg->state != msg->DUMMY);
}

Vector2d AtlasFallDetector::getICP() {
  auto com = this->model->centerOfMass<double>(1);
  Vector3d com_pos = com.value();
  Vector3d com_vel = com.gradient().value() * this->robot_state.qd;
  Vector2d icp = com_pos.head<2>() + std::sqrt((com_pos(2) - this->getSupportFootHeight()) / this->g) * com_vel.head<2>();
  return icp;
}

double AtlasFallDetector::getSupportFootHeight() {
  std::map<FootID, double> sole_zs;
  for (std::map<FootID, int>::iterator foot = this->foot_body_ids.begin(); foot != foot_body_ids.end(); ++foot) {
    Matrix3Xd contact_pts;
    this->model->getTerrainContactPoints(*this->model->bodies[foot->second], contact_pts);
    sole_zs[foot->first] = contact_pts.row(2).mean();
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
    Matrix3Xd contact_pts;
    this->model->getTerrainContactPoints(*this->model->bodies[foot->second], contact_pts);
    if (!this->foot_contact.at(foot->first)) {
      // If foot is out of contact, only use its center to define the support polygon
      for (int i=0; i < 3; ++i) {
        contact_pts(i, 0) = contact_pts.row(i).mean();
      }
      contact_pts.conservativeResize(3, 1);
    }
    all_contact_pts.conservativeResize(3, all_contact_pts.cols() + contact_pts.cols());
    all_contact_pts.block(0, all_contact_pts.cols() - contact_pts.cols(), 3, contact_pts.cols()) = contact_pts;
  }

  return all_contact_pts;
}

bool AtlasFallDetector::isICPCaptured() {
  Vector2d icp = this->getICP();
  Matrix3Xd support_pts = this->getVirtualSupportPolygon();
  return in_convex_hull(support_pts.topRows(2), icp);
}

int main(int argc, char** argv) {
  lcm::LCM lcm;
  if (!lcm.good()) {
    throw std::runtime_error("LCM is not good");
  }

  const char* drc_path = std::getenv("DRC_BASE");
  std::cout << "drc path: " << std::endl;
  if (!drc_path) {
    throw std::runtime_error("environment variable DRC_BASE is not set");
  }
  std::shared_ptr<RigidBodyManipulator> model(new RigidBodyManipulator(std::string(drc_path) + "/software/models/atlas_v5/model_minimal_contact.urdf"));
  model->setUseNewKinsol(true);
  model->compile();

  std::unique_ptr<AtlasFallDetector> fall_detector(new AtlasFallDetector(model));
  lcm.subscribe("FOOT_CONTACT_ESTIMATE", &AtlasFallDetector::handleFootContact, fall_detector.get());
  lcm.subscribe("EST_ROBOT_STATE", &AtlasFallDetector::handleRobotState, fall_detector.get());
  lcm.subscribe("CONTROLLER_STATUS", &AtlasFallDetector::handleControllerStatus, fall_detector.get());

  std::cout << "Atlas fall detector running" << std::endl;
  std::cout << "finished" << std::endl;
}
