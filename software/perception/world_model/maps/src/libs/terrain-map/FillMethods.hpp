#ifndef _terrain_map_FillMethods_hpp_
#define _terrain_map_FillMethods_hpp_

#include <memory>
#include <Eigen/Geometry>

namespace maps {
  class DepthImageView;
}

namespace terrainmap {

class FillMethods {
private:
  typedef std::shared_ptr<maps::DepthImageView> View;

public:
  static bool
  fillEntireView(View& ioView, const Eigen::Vector4d& iPlane);

  static bool
  fillBox(View& ioView, const Eigen::Vector3d& iPosition,
          const double iRadius, const Eigen::Vector4d& iPlane);

  static bool
  fillHolesIterative(View& ioView);

  static bool
  fillMissing(View& ioView, const Eigen::Vector4d& iPlane);

};

}

#endif
