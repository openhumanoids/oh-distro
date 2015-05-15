#ifndef _planeseg_PlaneFitter_hpp_

#include "Types.hpp"

namespace planeseg {

class PlaneFitter {
public:
  struct Result {
    bool mSuccess;
    Eigen::Vector4f mPlane;
    std::vector<int> mInliers;
    Eigen::Vector3f mCenterPoint;
    float mCurvature;
  };

public:
  PlaneFitter();
  ~PlaneFitter();

  void setMaxDistance(const float iDistance);
  void setCenterPoint(const Eigen::Vector3f& iPoint);
  void setMaxIterations(const int iIterations);
  void setRefineUsingInliers(const bool iVal);

  Result go(const std::vector<Eigen::Vector3f>& iPoints) const;

protected:
  template<typename T>
  Result solve(const std::vector<Eigen::Vector3f>& iPoints) const;

protected:
  Eigen::Vector3f mCenterPoint;
  float mMaxDistance;
  int mMaxIterations;
  bool mRefineUsingInliers;
};

}

#endif
