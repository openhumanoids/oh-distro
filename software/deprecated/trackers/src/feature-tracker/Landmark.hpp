#ifndef _tracking_Landmark_hpp_
#define _tracking_Landmark_hpp_

#include <Eigen/Geometry>

namespace tracking {

struct Landmark {
  int mId;
  int mPyramidLevel;
  int64_t mKeyFrameId;
  Eigen::Vector3f mPos3d;
  Eigen::Vector3f mNormal;
  Eigen::Vector2f mPosLeft;
  Eigen::Vector2f mPosRight;
  Eigen::Matrix3f mPosCov;
};

}

#endif
