#ifndef _tracking_PointMatcher_hpp_
#define _tracking_PointMatcher_hpp_

#include <vector>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "PointMatch.hpp"

namespace tracking {

class PointMatcher {
public:
  PointMatcher();

  PointMatch
  refine(const Eigen::Vector2f& iCurPos,
         const std::vector<cv::Mat>& iCurPyramid,
         const Eigen::Affine2f& iRefTransform,
         const std::vector<cv::Mat>& iRefPyramid,
         const int iPatchRadiusX, const int iPatchRadiusY,
         const Eigen::Vector2f& iConstraintDir=Eigen::Vector2f(0,0),
         const int iMaxIterations=10) const;

  // TODO: function for tracking large patch and mask

  // TODO: function for tracking via affine warp rather than translation only
};

}

#endif
