#ifndef _tracking_PointMatch_hpp_
#define _tracking_PointMatch_hpp_

#include <Eigen/Geometry>

namespace tracking {

struct PointMatch {
  Eigen::Vector2f mRefPos;
  Eigen::Vector2f mCurPos;
  float mScore;
};

}

#endif
