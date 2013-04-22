#ifndef _tracking_StereoCamera_hpp_
#define _tracking_StereoCamera_hpp_

#include "CameraModel.hpp"

namespace tracking {

class StereoCamera {
public:
  StereoCamera();

  void setLeftCamera(const CameraModel& iCamera);
  void setRightCamera(const CameraModel& iCamera);

  const CameraModel& getLeftCamera() const;
  const CameraModel& getRightCamera() const;

  void applyPose(const Eigen::Isometry3f& iPose);

  // TODO: utility functions
  // triangulate, project, get epipolar line, etc
  

protected:
  CameraModel mLeftCamera;
  CameraModel mRightCamera;
};

}

#endif
