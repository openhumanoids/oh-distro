#include "PlaneFitter.hpp"

#include <limits>

#include <drc_utils/RansacGeneric.hpp>

using namespace planeseg;

namespace {

struct SimpleProblemBase {
  struct Solution {
    Eigen::Vector4f mPlane;
    float mCurvature;
    Eigen::Vector3f mCenterPoint;
  };

  MatrixX3f mPoints;
  Eigen::Vector3f mCenterPoint;

  SimpleProblemBase(const std::vector<Eigen::Vector3f>& iPoints) {
    mPoints.resize(iPoints.size(),3);
    for (int i = 0; i < (int)iPoints.size(); ++i) {
      mPoints.row(i) = iPoints[i];
    }
  }
  void setCenterPoint(const Eigen::Vector3f& iPoint) { mCenterPoint = iPoint; }
  int getSampleSize() const { return 3; }
  int getNumDataPoints() const { return mPoints.rows(); }

  Solution estimate(const std::vector<int>& iIndices) const {
    Solution sol;
    const int n = iIndices.size();
    if (n == 3) {
      Eigen::Vector3f p1 = mPoints.row(iIndices[0]);
      Eigen::Vector3f p2 = mPoints.row(iIndices[1]);
      Eigen::Vector3f p3 = mPoints.row(iIndices[2]);
      sol.mPlane.head<3>() = ((p3-p1).cross(p2-p1)).normalized();
      sol.mPlane[3] = -sol.mPlane.head<3>().dot(p1);
      sol.mCurvature = 0;
      float centerDist = sol.mPlane.head<3>().dot(mCenterPoint) + sol.mPlane[3];
      if (std::abs(centerDist) > 0.02f)  sol.mPlane[3] = 1e10;
    }
    else {
      sol = estimateFull(iIndices);
    }
    return sol;
  }

  Solution estimateFull(const std::vector<int>& iIndices) const {
    Solution sol;
    const int n = iIndices.size();
    MatrixX3f data(n,3);
    for (int i = 0; i < n; ++i) data.row(i) = mPoints.row(iIndices[i]);
    Eigen::Vector3f avg = data.colwise().mean();
    data.rowwise() -= avg.transpose();
    auto svd = data.jacobiSvd(Eigen::ComputeFullV);
    sol.mPlane.head<3>() = svd.matrixV().col(2);
    sol.mPlane[3] = -sol.mPlane.head<3>().dot(avg);
    sol.mCurvature = svd.singularValues()[2]/n;
    sol.mCenterPoint = avg;
    return sol;
  }
  
  std::vector<double> computeSquaredErrors(const Solution& iSolution) const {
    const auto& plane = iSolution.mPlane;
    Eigen::VectorXf errors = (mPoints*plane.head<3>()).array() + plane[3];
    std::vector<double> errors2(errors.size());
    for (int i = 0; i < (int)errors2.size(); ++i) {
      errors2[i] = errors[i]*errors[i];
    }
    return errors2;
  }
};

struct SimpleProblem : public SimpleProblemBase {
  SimpleProblem(const std::vector<Eigen::Vector3f>& iPoints) :
    SimpleProblemBase(iPoints) {}

  int getSampleSize() const { return 2; }

  Solution estimate(const std::vector<int>& iIndices) const {
    Solution sol;
    const int n = iIndices.size();
    if (n == 2) {
      Eigen::Vector3f p1 = mPoints.row(iIndices[0]);
      Eigen::Vector3f p2 = mPoints.row(iIndices[1]);
      const Eigen::Vector3f& p3 = mCenterPoint;
      sol.mPlane.head<3>() = ((p3-p1).cross(p2-p1)).normalized();
      sol.mPlane[3] = -sol.mPlane.head<3>().dot(p1);
      sol.mCurvature = 0;
    }
    else {
      sol = estimateFull(iIndices);
    }
    return sol;
  }

};

}


PlaneFitter::
PlaneFitter() {
  setMaxDistance(0.01);
  setMaxIterations(100);
  setRefineUsingInliers(true);
  float badValue = std::numeric_limits<float>::infinity();
  setCenterPoint(Eigen::Vector3f(badValue, badValue, badValue));
}

PlaneFitter::
~PlaneFitter() {
}

void PlaneFitter::
setMaxDistance(const float iDistance) {
  mMaxDistance = iDistance;
}

void PlaneFitter::
setCenterPoint(const Eigen::Vector3f& iPoint) {
  mCenterPoint = iPoint;
}

void PlaneFitter::
setMaxIterations(const int iIterations) {
  mMaxIterations = iIterations;
}

void PlaneFitter::
setRefineUsingInliers(const bool iVal) {
  mRefineUsingInliers = iVal;
}

PlaneFitter::Result PlaneFitter::
go(const std::vector<Eigen::Vector3f>& iPoints) const {
  if (std::isinf(mCenterPoint[0])) return solve<SimpleProblemBase>(iPoints);
  else return solve<SimpleProblem>(iPoints);
}

template<typename T>
PlaneFitter::Result PlaneFitter::
solve(const std::vector<Eigen::Vector3f>& iPoints) const {
  Result result;
  drc::RansacGeneric<T> ransac;
  ransac.setMaximumError(mMaxDistance);
  ransac.setRefineUsingInliers(mRefineUsingInliers);
  ransac.setMaximumIterations(mMaxIterations);

  SimpleProblem problem(iPoints);
  problem.setCenterPoint(mCenterPoint);

  auto res = ransac.solve(problem);
  result.mSuccess = res.mSuccess;
  result.mPlane = res.mSolution.mPlane;
  result.mCenterPoint = res.mSolution.mCenterPoint;
  result.mInliers = res.mInliers;
  result.mCurvature = res.mSolution.mCurvature;

  return result;
}
