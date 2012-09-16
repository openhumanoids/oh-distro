#ifndef _DenseGrid3d_hpp_
#define _DenseGrid3d_hpp_

#include <vector>
#include <Eigen/Geometry>

class DenseGrid3d {
public:
  DenseGrid3d();

  void setDimensions(const int iX, const int iY, const int iZ);
  Eigen::Vector3i getDimensions() const { return mDimensions; }

  void setToLocal(const Eigen::Affine3d& iTransform);
  Eigen::Affine3d getToLocal() const { return mToLocalTransform; }

  int64_t toIndex(const int iX, const int iY, const int iZ) const {
    return iZ*mPlaneStrideBytes + iY*mRowStrideBytes + iX;
  }

  Eigen::Vector3i fromIndex(const int64_t iIndex) const {
    int index = iIndex;
    Eigen::Vector3i pt;
    pt[2] = index / mPlaneStrideBytes;
    index -= pt[2]*mPlaneStrideBytes;
    pt[1] = index / mRowStrideBytes;
    index -= pt[1]*mRowStrideBytes;
    pt[0] = index;
    return pt;
  }

  void clear();

  int8_t& operator()(const int64_t iIndex) {
    return mData[iIndex];
  }
  int8_t operator()(const int64_t iIndex) const {
    return mData[iIndex];
  }

  int8_t& operator()(const int iX, const int iY, const int iZ) {
    return mData[toIndex(iX, iY, iZ)];
  }
  int8_t operator()(const int iX, const int iY, const int iZ) const {
    return mData[toIndex(iX, iY, iZ)];
  }

  int8_t& operator()(const Eigen::Vector3i& iPoint) {
    return mData[toIndex(iPoint[0], iPoint[1], iPoint[2])];
  }
  int8_t operator()(const Eigen::Vector3i& iPoint) const {
    return mData[toIndex(iPoint[0], iPoint[1], iPoint[2])];
  }

  bool checkBounds(const int iX, const int iY, const int iZ) const {
    return ((iX >= 0) && (iX < mDimensions[0]) &&
            (iY >= 0) && (iY < mDimensions[1]) &&
            (iZ >= 0) && (iZ < mDimensions[2]));
  }

  bool checkBounds(const Eigen::Vector3i& iPoint) {
    return checkBounds(iPoint[0], iPoint[1], iPoint[2]);
  }

  bool addPoint(const Eigen::Vector3d& iPoint) {
    Eigen::Vector3d transformed = mFromLocalTransform*iPoint;
    Eigen::Vector3i pt = transformed.cast<int>();
    if (!checkBounds(pt)) {
      return false;
    }
    (*this)(pt) = 1;
    return true;
  }

  bool removePoint(const Eigen::Vector3d& iPoint) {
    Eigen::Vector3d transformed = mFromLocalTransform*iPoint;
    Eigen::Vector3i pt = transformed.cast<int>();
    if (!checkBounds(pt)) {
      return false;
    }
    (*this)(pt) = -1;
    return true;
  }

protected:
  Eigen::Vector3i mDimensions;
  int mPlaneStrideBytes;
  int mRowStrideBytes;
  Eigen::Affine3d mToLocalTransform;
  Eigen::Affine3d mFromLocalTransform;
  std::vector<int8_t> mData;
};

#endif
