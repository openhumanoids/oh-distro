#ifndef _tracking_TrackedObject_hpp_
#define _tracking_TrackedObject_hpp_

#include <memory>
#include <Eigen/Geometry>

#include "Landmark.hpp"

namespace tracking {

struct TrackedObject {
  typedef std::shared_ptr<TrackedObject> Ptr;

  struct State {
    int64_t mTimestamp;
    Eigen::Isometry3f mPose;
  };

  int mId;
  State mOriginalState;
  State mCurrentState;

  std::vector<Landmark> mLandmarks;
};

}

#endif
