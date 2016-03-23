#include "FootContactDriver.hpp"

FootContactDriver::FootContactDriver(RobotPropertyCache rpc) {
  l_foot_id = rpc.foot_ids[Side::LEFT];
  r_foot_id = rpc.foot_ids[Side::RIGHT];
}

void FootContactDriver::decode(const drc::foot_contact_estimate_t *msg, Eigen::Matrix<bool, Eigen::Dynamic, 1> &contact_force_detected) {
  contact_force_detected(r_foot_id) = msg->right_contact > 0.5;
  contact_force_detected(l_foot_id) = msg->left_contact > 0.5;
}



