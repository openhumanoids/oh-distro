#ifndef _drc_LidarUtils_hpp_
#define _drc_LidarUtils_hpp_

#include <vector>
#include <Eigen/Geometry>

namespace drc {
class LidarUtils {
public:
  static bool
  interpolateScan(const std::vector<float>& iRanges,
                  const double iTheta0, const double iThetaStep,
                  const Eigen::Isometry3d& iPose0,
                  const Eigen::Isometry3d& iPose1,
                  std::vector<Eigen::Vector3f>& oPoints);
};
}

#endif
