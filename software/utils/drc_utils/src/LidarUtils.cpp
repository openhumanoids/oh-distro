#include "LidarUtils.hpp"

#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>

using namespace drc;

bool LidarUtils::
interpolateScan(const std::vector<float>& iRanges,
                const double iTheta0, const double iThetaStep,
                const Eigen::Isometry3d& iPose0,
                const Eigen::Isometry3d& iPose1,
                std::vector<Eigen::Vector3f>& oPoints) {
  const int n = iRanges.size();
  if (n < 2) return false;
  const double tStep = 1.0/(n-1);
  Eigen::Quaterniond q0(iPose0.linear());
  Eigen::Quaterniond q1(iPose1.linear());
  Eigen::Vector3d pos0(iPose0.translation());
  Eigen::Vector3d pos1(iPose1.translation());
  oPoints.resize(n);
  double t = 0;
  double theta = iTheta0;
  for (int i = 0; i < n; ++i, t += tStep, theta += iThetaStep) {
    Eigen::Quaterniond q = q0.slerp(t,q1);
    Eigen::Vector3d pos = (1-t)*pos0 + t*pos1;
    Eigen::Vector3d pt = iRanges[i]*Eigen::Vector3d(cos(theta), sin(theta), 0);
    oPoints[i] = (q*pt + pos).cast<float>();
  }
  return true;
}
