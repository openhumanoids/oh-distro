#include "LidarScan.hpp"

using namespace maps;

LidarScan::
LidarScan() {
  mTimestamp = 0;
  mThetaMin = 0;
  mThetaStep = 0;
  mPose = Eigen::Isometry3f::Identity();
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
setAngles(const float iThetaMin, const float iThetaStep) {
  mThetaMin = iThetaMin;
  mThetaStep = iThetaStep;
}

void LidarScan::
setPose(const Eigen::Isometry3f& iPose) {
  mPose = iPose;
}

int64_t LidarScan::
getTimestamp() const {
  return mTimestamp;
}

const std::vector<float>& LidarScan::
getRanges() const {
  return mRanges;
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
  return mPose;
}

Eigen::Vector3f LidarScan::
getVector(const int iIndex) const {
  Eigen::Vector3f pt;
  double theta = mThetaMin + iIndex*(double)mThetaStep;
  double range = mRanges[iIndex];
  pt << range*cos(theta), range*sin(theta), 0;
  return pt;
}

maps::PointType LidarScan::
getPoint(const int iIndex) const {
  maps::PointType pt;
  double theta = mThetaMin + iIndex*(double)mThetaStep;
  double range = mRanges[iIndex];
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
  oPoints.mCloud->resize(n);
  for (int i = 0; i < n; ++i) {
    if (mRanges[i] >= 0) (*oPoints.mCloud)[i] = getPoint(i);
  }
  oPoints.mCloud->sensor_origin_.head<3>() = mPose.translation();
  oPoints.mCloud->sensor_origin_[3] = 1;
  oPoints.mCloud->sensor_orientation_ = mPose.linear();
}
