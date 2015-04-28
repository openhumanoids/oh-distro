#include "RobustNormalEstimator.hpp"

#include <drc_utils/RansacGeneric.hpp>
#include <pcl/search/kdtree.h>

#include "Types.hpp"

using namespace planeseg;

struct RobustNormalEstimator::SimpleProblem {
  struct Solution {
    Eigen::Vector4f mPlane;
    float mCurvature;
  };

  MatrixX3f mPoints;

  SimpleProblem(const std::vector<Eigen::Vector3f>& iPoints) {
    mPoints.resize(iPoints.size(),3);
    for (int i = 0; i < (int)iPoints.size(); ++i) {
      mPoints.row(i) = iPoints[i];
    }
  }
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
    }
    else {
      MatrixX3f data(n,3);
      for (int i = 0; i < n; ++i) {
        data.row(i) = mPoints.row(iIndices[i]);
      }
      Eigen::Vector3f avg = data.colwise().mean();
      data.rowwise() -= avg.transpose();
      auto svd = data.jacobiSvd(Eigen::ComputeFullV);
      sol.mPlane.head<3>() = svd.matrixV().col(2);
      sol.mPlane[3] = -sol.mPlane.head<3>().dot(avg);
      sol.mCurvature = svd.singularValues()[2]/n;
    }
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

RobustNormalEstimator::
RobustNormalEstimator() {
  setRadius(0.1);
  setMaxEstimationError(0.01);
  setMaxCenterError(0.02);
  setMaxIterations(100);
}
  
void RobustNormalEstimator::
setRadius(const float iRadius) {
  mRadius = iRadius;
}

void RobustNormalEstimator::
setMaxEstimationError(const float iDist) {
  mMaxEstimationError = iDist;
}

void RobustNormalEstimator::
setMaxCenterError(const float iDist) {
  mMaxCenterError = iDist;
}

void RobustNormalEstimator::
setMaxIterations(const int iIters) {
  mMaxIterations = iIters;
}

bool RobustNormalEstimator::
go(const LabeledCloud::Ptr& iCloud, NormalCloud& oNormals) {

  // ransac
  drc::RansacGeneric<SimpleProblem> ransac;
  ransac.setMaximumError(0.01);
  ransac.setRefineUsingInliers(true);
  ransac.setMaximumIterations(100);
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(1000);

  // kd tree
  pcl::search::KdTree<pcl::PointXYZL>::Ptr tree
    (new pcl::search::KdTree<pcl::PointXYZL>());
  tree->setInputCloud(iCloud);
  std::vector<int> indices;
  std::vector<float> distances;

  // loop
  const int n = iCloud->size();
  oNormals.width = n;
  oNormals.height = 0;
  oNormals.resize(n);
  oNormals.is_dense = false;
  for (int i = 0; i < n; ++i) {
    tree->radiusSearch(i, 0.1, indices, distances);
    pts.clear();
    for (const auto idx : indices) {
      pts.push_back(iCloud->points[idx].getVector3fMap());
    }
    auto& norm = oNormals.points[i];
    norm.normal_x = norm.normal_y = norm.normal_z = 0;
    norm.curvature = -1;
    if (pts.size() < 3) continue;

    // solve for plane
    SimpleProblem problem(pts);
    auto res = ransac.solve(problem);
    Eigen::Vector3f normal = res.mSolution.mPlane.head<3>();
    Eigen::Vector3f pt = iCloud->points[i].getVector3fMap();
    if (std::abs(normal.dot(pt) + res.mSolution.mPlane[3]) >
        mMaxCenterError) continue;
    if (normal[2]<0) normal = -normal;
    norm.normal_x = normal[0];
    norm.normal_y = normal[1];
    norm.normal_z = normal[2];
    norm.curvature = res.mSolution.mCurvature;
  }

  return true;
}

