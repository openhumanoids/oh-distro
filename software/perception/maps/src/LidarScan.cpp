#include "LidarScan.hpp"

using namespace maps;

LidarScan::
LidarScan() {
  mTimestamp = 0;
  mThetaMin = 0;
  mThetaStep = 0;
  mPoseStart = Eigen::Isometry3f::Identity();
  mPoseEnd = Eigen::Isometry3f::Identity();
}

void LidarScan::
setTimestamp(const int64_t iTimestamp) {
  mTimestamp = iTimestamp;
}

void LidarScan::
setRanges(const std::vector<float>& iRanges) {
  mRanges = iRanges;
}

void LidarScan::
setIntensities(const std::vector<float>& iIntensities) {
  mIntensities = iIntensities;
}

void LidarScan::
setAngles(const float iThetaMin, const float iThetaStep) {
  mThetaMin = iThetaMin;
  mThetaStep = iThetaStep;
}

void LidarScan::
setPose(const Eigen::Isometry3f& iPose) {
  mPoseEnd = iPose;
}

void LidarScan::
setPoses(const Eigen::Isometry3f& iStartPose,
         const Eigen::Isometry3f& iEndPose) {
  mPoseStart = iStartPose;
  mPoseEnd = iEndPose;
}

int64_t LidarScan::
getTimestamp() const {
  return mTimestamp;
}

int LidarScan::
getNumRanges() const {
  return mRanges.size();
}

int LidarScan::
getNumIntensities() const {
  return mIntensities.size();
}

const std::vector<float>& LidarScan::
getRanges() const {
  return mRanges;
}

const std::vector<float>& LidarScan::
getIntensities() const {
  return mIntensities;
}

float LidarScan::
getThetaMin() const {
  return mThetaMin;
}

float LidarScan::
getThetaStep() const {
  return mThetaStep;
}

Eigen::Isometry3f
LidarScan::getPose() const {
  return mPoseEnd;
}

Eigen::Isometry3f
LidarScan::getStartPose() const {
  return mPoseStart;
}

Eigen::Isometry3f
LidarScan::getEndPose() const {
  return mPoseEnd;
}

Eigen::Vector3f LidarScan::
getVector(const int iIndex) const {
  Eigen::Vector3f pt;
  float theta = mThetaMin + iIndex*mThetaStep;
  float range = mRanges[iIndex];
  pt << range*cos(theta), range*sin(theta), 0;
  return pt;
}

maps::PointType LidarScan::
getPoint(const int iIndex) const {
  maps::PointType pt;
  float theta = mThetaMin + iIndex*mThetaStep;
  float range = mRanges[iIndex];
  pt.x = range*cos(theta);
  pt.y = range*sin(theta);
  pt.z = 0;
  return pt;
}

void LidarScan::
get(maps::PointSet& oPoints) const {
  oPoints.mTimestamp = mTimestamp;
  int n = mRanges.size();
  oPoints.mCloud.reset(new maps::PointCloud());
  oPoints.mCloud->reserve(n);
  for (int i = 0; i < n; ++i) {
    if (mRanges[i] > 0) (*oPoints.mCloud).push_back(getPoint(i));
  }
  oPoints.mCloud->sensor_origin_.head<3>() = mPoseEnd.translation();
  oPoints.mCloud->sensor_origin_[3] = 1;
  oPoints.mCloud->sensor_orientation_ = mPoseEnd.linear();
}

void LidarScan::
get(maps::PointCloud& oCloud, const bool iInterp) const {
  const int n = mRanges.size();
  oCloud.reserve(n);
  oCloud.clear();

  if (iInterp) {
    Eigen::Quaternionf q1(mPoseStart.rotation());
    Eigen::Quaternionf q2(mPoseEnd.rotation());
    Eigen::Vector3f T1(mPoseStart.translation());
    Eigen::Vector3f T2(mPoseEnd.translation());
    for (int i = 0; i < n; ++i) {
      if (mRanges[i] <= 0) continue;
      float alpha = (float)i/(n-1);
      Eigen::Vector3f T = (1-alpha)*T1 + alpha*T2;
      Eigen::Quaternionf q = q1.slerp(alpha, q2);
      Eigen::Vector3f pt = q*getVector(i) + T;
      PointType cloudPt;
      cloudPt.x = pt[0];  cloudPt.y = pt[1];  cloudPt.z = pt[2];
      oCloud.push_back(cloudPt);
    }
  }
  else {
    Eigen::Matrix3f R = mPoseEnd.rotation();
    Eigen::Vector3f T = mPoseEnd.translation();
    for (int i = 0; i < n; ++i) {
      if (mRanges[i] <= 0) continue;
      Eigen::Vector3f pt = R*getVector(i) + T;
      PointType cloudPt;
      cloudPt.x = pt[0];  cloudPt.y = pt[1];  cloudPt.z = pt[2];
      oCloud.push_back(cloudPt);
    }
  }
  oCloud.is_dense = false;
  oCloud.width = oCloud.size();
  oCloud.height = 1;
}
