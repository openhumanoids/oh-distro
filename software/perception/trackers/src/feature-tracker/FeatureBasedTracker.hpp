#ifndef _tracking_FeatureBasedTracker_hpp_
#define _tracking_FeatureBasedTracker_hpp_

#include <memory>
#include <Eigen/Geometry>
#include "TrackedObject.hpp"

namespace cv {
  class Mat;
}

namespace tracking {

class StereoCamera;

class FeatureBasedTracker {
protected:
  struct Helper;

public:
  FeatureBasedTracker();

  void clear();

  void setCamera(const StereoCamera& iCamera);
  void setPatchRadius(const int iRadiusX, const int iRadiusY);
  void setMinMatchScore(const float iScore);

  // must be called before initialize() or update() on respective frame
  void setData(const int64_t iTime, const cv::Mat& iLeftImage,
               const cv::Mat& iRightImage, const cv::Mat& iDisparity,
               const Eigen::Isometry3f& iSensorPose);

  bool initialize(const int iId, const cv::Mat& iMask,
                  const Eigen::Isometry3f& iObjectPose);

  bool update();

  std::vector<int> getAllTrackIds() const;
  TrackedObject::State getCurrentState(const int iId) const;

protected:
  std::shared_ptr<Helper> mHelper;
};

}

#endif
