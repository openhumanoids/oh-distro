#ifndef _maps_VoxelGrid_hpp_
#define _maps_VoxelGrid_hpp_

#include <vector>
#include <Eigen/Geometry>

namespace maps {

class VoxelGrid {
public:
  VoxelGrid();

  void setDimensions(const int iX, const int iY, const int iZ);
  Eigen::Vector3i getDimensions() const { return mDimensions; }

  void setTransform(const Eigen::Affine3f& iTransform);
  Eigen::Affine3f getTransform() const { return mTransform; }

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
  const int8_t operator()(const int64_t iIndex) const {
    return mData[iIndex];
  }

  int8_t& operator()(const int iX, const int iY, const int iZ) {
    return mData[toIndex(iX, iY, iZ)];
  }
  const int8_t operator()(const int iX, const int iY, const int iZ) const {
    return mData[toIndex(iX, iY, iZ)];
  }

  int8_t& operator()(const Eigen::Vector3i& iPoint) {
    return mData[toIndex(iPoint[0], iPoint[1], iPoint[2])];
  }
  const int8_t operator()(const Eigen::Vector3i& iPoint) const {
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

  bool addPoint(const Eigen::Vector3f& iPoint) {
    Eigen::Vector3f transformed = mTransformInv*iPoint;
    Eigen::Vector3i pt = transformed.cast<int>();
    if (!checkBounds(pt)) {
      return false;
    }
    (*this)(pt) = 1;
    return true;
  }

  bool removePoint(const Eigen::Vector3f& iPoint) {
    Eigen::Vector3f transformed = mTransformInv*iPoint;
    Eigen::Vector3i pt = transformed.cast<int>();
    if (!checkBounds(pt)) {
      return false;
    }
    (*this)(pt) = -1;
    return true;
  }

  bool traceRay(const Eigen::Vector3f& iOrigin,
                const Eigen::Vector3f& iDirection,
                std::vector<int64_t>& oIndices);

  // API functions to add
  // multiresolution capability (set num levels)
  //   or have pyramid sitting above; this could just be basic accessor
  // castRay or equivalent
  // insertRay or equivalent
  // intersect ray with volume to find start and end points (or none)
  // efficient bresenham walking of voxels along ray (templated?)
  //   integer math
  //   
  // splatPoint(val) - occupied or unoccupied
  // (decide on what the values mean)
  // serialization (can pack bits if just occupancy is desired)
  //   = factor of 4 or 8 depending on whether unobserved values need to be stored
  //               (or, can encode just occupied cells as (short,short,short) triples with byte value (or no value if just occupancy))
  // change detection
  // depth projection onto plane
  // depth projection onto cardinal planes

protected:
  Eigen::Vector3i mDimensions;
  int mPlaneStrideBytes;
  int mRowStrideBytes;
  Eigen::Affine3f mTransform;
  Eigen::Affine3f mTransformInv;
  std::vector<int8_t> mData;
};

}

#endif
