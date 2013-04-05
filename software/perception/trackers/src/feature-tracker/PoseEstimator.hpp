#ifndef _tracking_PoseEstimator_hpp_
#define _tracking_PoseEstimator_hpp_

#include <vector>
#include <Eigen/Geometry>

class PoseEstimator {
public:
  PoseEstimator();

  void setErrorThreshold(const float iThresh);

  bool leastSquares(const std::vector<Eigen::Vector3f>& iRefPoints,
                    const std::vector<Eigen::Vector3f>& iCurPoints,
                    Eigen::Isometry3f& oPose) const;

  bool ransac(const std::vector<Eigen::Vector3f>& iRefPoints,
              const std::vector<Eigen::Vector3f>& iCurPoints,
              Eigen::Isometry3f& oPose, std::vector<int>& oInliers) const;
  

protected:
  float mErrorThreshold;
};

#endif
