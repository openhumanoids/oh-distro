#ifndef _planeseg_BlockFitter_hpp_
#define _planeseg_BlockFitter_hpp_

#include "Types.hpp"

namespace planeseg {

class BlockFitter {
public:
  struct Block {
    Eigen::Vector3f mSize;
    Eigen::Isometry3f mPose;
  };
  struct Result {
    std::vector<Block> mBlocks;
  };

public:
  BlockFitter();

  void setSensorPose(const Eigen::Vector3f& iOrigin,
                     const Eigen::Vector3f& iLookDir);
  void setBlockDimensions(const Eigen::Vector3f& iDimensions);
  void setDownsampleResolution(const float iRes);
  void setRemoveGround(const bool iVal);
  void setHeightBand(const float iMinHeight, const float iMaxHeight);
  void setMaxRange(const float iRange);
  void setMaxAngleFromHorizontal(const float iDegrees);
  void setCloud(const LabeledCloud::Ptr& iCloud);
  void setDebug(const bool iVal);

  Result go();

protected:
  Eigen::Vector3f mOrigin;
  Eigen::Vector3f mLookDir;
  Eigen::Vector3f mBlockDimensions;
  float mDownsampleResolution;
  bool mRemoveGround;
  float mMinHeightAboveGround;
  float mMaxHeightAboveGround;
  float mMaxRange;
  float mMaxAngleFromHorizontal;
  LabeledCloud::Ptr mCloud;
  bool mDebug;
};

}

#endif
