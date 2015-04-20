#ifndef _maps_Utils_hpp_
#define _maps_Utils_hpp_

#include <Eigen/Geometry>
#include <vector>

namespace maps {

class Utils {

public:
  static bool isOrthographic(const Eigen::Matrix4f& iMatrix);
  static bool composeViewMatrix(Eigen::Projective3f& oMatrix,
                                const Eigen::Matrix3f& iCalib,
                                const Eigen::Isometry3f& iPose,
                                const bool iIsOrthographic);
  static bool factorViewMatrix(const Eigen::Projective3f& iMatrix,
                               Eigen::Matrix3f& oCalib,
                               Eigen::Isometry3f& oPose,
                               bool& oIsOrthographic);

  static uint64_t rand64();
};

}

#endif
