//
// Created by manuelli on 2/10/17.
//
#include "bot_core/position_3d_t.hpp"
#include <Eigen/Dense>

namespace plan_eval{
namespace utils{
  Eigen::Isometry3d IsometryFromPosition3dMsg(const bot_core::position_3d_t& msg){
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() << msg.translation.x, msg.translation.y, msg.translation.z;
    Eigen::Quaternion<double> quat = Eigen::Quaternion<double>(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
    pose.rotate(quat);
    return pose;
  }
}// utils
}// plan_eval
