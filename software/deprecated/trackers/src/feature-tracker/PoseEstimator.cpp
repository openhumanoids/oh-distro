#include "PoseEstimator.hpp"

#include "RansacGeneric.hpp"

using namespace tracking;

PoseEstimator::
PoseEstimator() {
  setErrorThreshold(0.03);
}

void PoseEstimator::
setErrorThreshold(const float iThresh) {
  mErrorThreshold = iThresh;
}

bool PoseEstimator::
leastSquares(const std::vector<Eigen::Vector3f>& iRefPoints,
             const std::vector<Eigen::Vector3f>& iCurPoints,
             Eigen::Isometry3f& oPose) const {
  const size_t n = iRefPoints.size();
  if ((n < 3) || (n != iCurPoints.size())) return false;

  // initialize pose object
  oPose = Eigen::Isometry3f::Identity();

  // compute center of mass of each point set
  Eigen::Vector3f meanRef(0,0,0), meanCur(0,0,0);
  for (size_t i = 0; i < n; ++i) {
    meanRef += iRefPoints[i];
    meanCur += iCurPoints[i];
  }
  meanRef /= n;
  meanCur /= n;

  // determine best rotation using procrustes method
  Eigen::MatrixXf refMatrix(n, 3);
  Eigen::MatrixXf curMatrix(n, 3);
  for (size_t i = 0; i < n; ++i) {
    refMatrix.row(i) = iRefPoints[i] - meanRef;
    curMatrix.row(i) = iCurPoints[i] - meanCur;
  }
  Eigen::Matrix3f matx = refMatrix.transpose()*curMatrix;
  Eigen::JacobiSVD<Eigen::Matrix3f> svd;
  svd.compute(matx, Eigen::ComputeFullU | Eigen::ComputeFullV);
  oPose.linear() = svd.matrixU()*svd.matrixV().transpose();

  // translation vector
  oPose.translation() = meanRef - oPose.linear()*meanCur;

  return true;
}

bool PoseEstimator::
ransac(const std::vector<Eigen::Vector3f>& iRefPoints,
       const std::vector<Eigen::Vector3f>& iCurPoints,
       Eigen::Isometry3f& oPose, std::vector<int>& oInliers) const {

  if (iRefPoints.size() != iCurPoints.size()) return false;
  if (iRefPoints.size() < 3) return false;

  struct Problem {
    typedef Eigen::Isometry3f Solution;

    PoseEstimator* mEstimator;
    std::vector<Eigen::Vector3f>* mRefPoints;
    std::vector<Eigen::Vector3f>* mCurPoints;

    Problem(const PoseEstimator* iEstimator) :
      mEstimator(const_cast<PoseEstimator*>(iEstimator)) {}

    int getNumDataPoints() const { return mRefPoints->size(); }
    int getSampleSize() const { return 3; }

    Eigen::Isometry3f estimate(const std::vector<int> iIndices) const {
      Eigen::Isometry3f pose;
      std::vector<Eigen::Vector3f> refPoints(iIndices.size());
      std::vector<Eigen::Vector3f> curPoints(iIndices.size());      
      for (size_t i = 0; i < iIndices.size(); ++i) {
        refPoints[i] = (*mRefPoints)[iIndices[i]];
        curPoints[i] = (*mCurPoints)[iIndices[i]];
      }
      mEstimator->leastSquares(refPoints, curPoints, pose);
      return pose;
    }

    std::vector<double>
    computeSquaredErrors(const Eigen::Isometry3f& iPose) const {
      size_t n = mRefPoints->size();
      std::vector<double> errors2(n);
      std::vector<Eigen::Vector3f>& refPoints = *mRefPoints;
      std::vector<Eigen::Vector3f>& curPoints = *mCurPoints;
      for (size_t i = 0; i < n; ++i) {
        Eigen::Vector3f error = refPoints[i] - iPose*curPoints[i];
        errors2[i] = error.squaredNorm();
      }
      return errors2;
    }
  };

  Problem problem(this);
  problem.mRefPoints = const_cast<std::vector<Eigen::Vector3f>*>(&iRefPoints);
  problem.mCurPoints = const_cast<std::vector<Eigen::Vector3f>*>(&iCurPoints);

  RansacGeneric<Problem> ransacObj;
  ransacObj.setMaximumError(mErrorThreshold);
  ransacObj.setRefineUsingInliers(true);
  RansacGeneric<Problem>::Result result = ransacObj.solve(problem);
  oPose = result.mSolution;
  oInliers = result.mInliers;

  return result.mSuccess;
}
  
