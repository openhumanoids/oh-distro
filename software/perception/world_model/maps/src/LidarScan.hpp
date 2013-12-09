#ifndef _maps_LidarScan_hpp_
#define _maps_LidarScan_hpp_

#include "Types.hpp"

namespace maps {

class LidarScan {
public:
  typedef std::shared_ptr<LidarScan> Ptr;
public:
  LidarScan();

  void setTimestamp(const int64_t iTimestamp);
  void setRanges(const std::vector<float>& iRanges);
  void setAngles(const float iThetaMin, const float iThetaStep);
  void setPose(const Eigen::Isometry3f& iPose);

  int64_t getTimestamp() const;
  const std::vector<float>& getRanges() const;
  float range(const int iIndex) const { return mRanges[iIndex]; }
  float& range(const int iIndex) { return mRanges[iIndex]; }
  float getThetaMin() const;
  float getThetaStep() const;
  Eigen::Isometry3f getPose() const;

  Eigen::Vector3f getVector(const int iIndex) const;
  maps::PointType getPoint(const int iIndex) const;
  void get(maps::PointSet& oPoints) const;
  

protected:
  int64_t mTimestamp;
  std::vector<float> mRanges;
  float mThetaMin;
  float mThetaStep;
  Eigen::Isometry3f mPose;
};

}

#endif
