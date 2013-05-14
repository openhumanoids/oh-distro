#ifndef _maps_VoxelGrid_hpp_
#define _maps_VoxelGrid_hpp_

#include <vector>
#include <memory>
#include <Eigen/Geometry>

// TODO TEMP
#include <iostream>

namespace maps {

template<typename T>
class VoxelGrid {
public:
  typedef std::shared_ptr<VoxelGrid<T> > Ptr;
public:
  VoxelGrid();

  void setDimensions(const int iX, const int iY, const int iZ);
  Eigen::Vector3i getDimensions() const { return mDimensions; }

  void setTransform(const Eigen::Affine3f& iTransform);
  Eigen::Affine3f getTransform() const { return mTransform; }

  void clear(const T& iValue = 0);

  T& operator()(const int iX, const int iY, const int iZ) {
    return mData[iX + mDimensions[0]*(iY + mDimensions[1]*iZ)];
  }
  const T operator()(const int iX, const int iY, const int iZ) const {
    return mData[iX + mDimensions[0]*(iY + mDimensions[1]*iZ)];
  }

  T& operator()(const Eigen::Vector3i& iPoint) {
    return (*this)(iPoint[0], iPoint[1], iPoint[2]);
  }
  const T operator()(const Eigen::Vector3i& iPoint) const {
    return (*this)(iPoint[0], iPoint[1], iPoint[2]);
  }

  bool checkBounds(const int iX, const int iY, const int iZ) const {
    return ((iX >= 0) && (iX < mDimensions[0]) &&
            (iY >= 0) && (iY < mDimensions[1]) &&
            (iZ >= 0) && (iZ < mDimensions[2]));
  }

  bool checkBounds(const Eigen::Vector3i& iPoint) {
    return checkBounds(iPoint[0], iPoint[1], iPoint[2]);
  }

  template<typename BoxOperator>
  void operateRecursive(BoxOperator& iOperator) {
    Eigen::Vector3i p1(0,0,0), p2(mDimensions-Eigen::Vector3i(1,1,1));
    Eigen::Vector3i boxSize = p2-p1;
    return operateRecurse(p1, p2, boxSize, iOperator);
  }

  template<typename RayOperator>
  bool traceRay(const Eigen::Vector3f& iOrigin,
                const Eigen::Vector3f& iEndPoint,
                RayOperator& iOperator) {
    Eigen::Vector3f p1 = mTransformInv*iOrigin;
    Eigen::Vector3f p2 = mTransformInv*iEndPoint;
    Eigen::Vector3f dir = p2 - p1;
    Eigen::Vector3f origin = p1;
    Eigen::Vector3f dims = mDimensions.cast<float>() - Eigen::Vector3f(1,1,1);
    float t1(0), t2(1);

    // clip against voxel bounds
    bool result =
      clipOriginPlane<0>(origin, dir, p1, p2, t1, t2) &&       //  1  0  0  0
      clipOriginPlane<1>(origin, dir, p1, p2, t1, t2) &&       //  0  1  0  0
      clipOriginPlane<2>(origin, dir, p1, p2, t1, t2) &&       //  0  0  1  0
      clipBoundPlane<0>(origin, dir, dims, p1, p2, t1, t2) &&  // -1  0  0 xmax
      clipBoundPlane<1>(origin, dir, dims, p1, p2, t1, t2) &&  //  0 -1  0 ymax
      clipBoundPlane<2>(origin, dir, dims, p1, p2, t1, t2);    //  0  0 -1 zmax
    if (!result) {
      return false;
    }

    // TODO: 3d bresenham
    // for now, just naive drawing of line from p1 to p2
    // TODO std::cout << "p2p2 " << p1.transpose() << "      " << p2.transpose() << std::endl;

    // find fastest-changing direction
    Eigen::Vector3f ad = dir.cwiseAbs();
    static const int lut[] = { 2, 0, 1, 0 };
    int bits = ((ad[0] >= ad[1]) && (ad[0] >= ad[2])) |
      (((ad[1] >= ad[0]) && (ad[1] >= ad[2])) << 1);
    int maxDim = lut[bits];
    Eigen::Vector3f step = dir/ad[maxDim];

    const int kFracBits = 8;

    // TODO: adjust p1 and p2 to lie on integer boundaries
    // TODO: compute remainder
    int numSteps = (p2-p1)[maxDim]+1;  // TODO: round appropriately
    Eigen::Vector3f p = p1;
    static const Eigen::Vector3f half(0.5,0.5,0.5);
    Eigen::Vector3i pInt;
    for (int i = 0; i < numSteps; ++i) {
      pInt = (p+half).cast<int>();
      iOperator(pInt, *this);
      // TODO std::cout << "pt " << p.transpose() << std::endl;
      p += step;
    }
    return true;
  }

protected:
  template<typename BoxOperator>
  void operateRecurse(const Eigen::Vector3i& iP1, const Eigen::Vector3i& iP2,
                      const Eigen::Vector3i& iBoxSize, BoxOperator& iOperator) {
    // base case; end recursion
    if (!iOperator(iP1, iP2, *this) ||
        ((iP1[0]==iP2[0]) && (iP1[1]==iP2[1]) && (iP1[2]==iP2[2]))) {
      return;
    }

    // determine which dimension to subdivide
    static const int lut[] = { 2, 0, 1, 0 };
    int bits =
      ((iBoxSize[0] >= iBoxSize[1]) && (iBoxSize[0] >= iBoxSize[2])) |
      (((iBoxSize[1] >= iBoxSize[0]) && (iBoxSize[1] >= iBoxSize[2])) << 1);
    int maxDim = lut[bits];

    // subdivide and recurse
    Eigen::Vector3i p2(iP2), p3(iP1), boxSize(iBoxSize);
    p2[maxDim] = (p2[maxDim]+iP1[maxDim])>>1;
    p3[maxDim] = p2[maxDim]+1;
    boxSize[maxDim] >>= 1;
    operateRecurse(iP1, p2, boxSize, iOperator);
    operateRecurse(p3, iP2, boxSize, iOperator);
  }


  template<int N>
  static inline bool clipOriginPlane(const Eigen::Vector3f& iOrigin,
                                     const Eigen::Vector3f& iDirection,
                                     Eigen::Vector3f& oP1, Eigen::Vector3f& oP2,
                                     float& oDist1, float& oDist2) {
    float dist1(oP1[N]), dist2(oP2[N]);
    if ((dist1 < 0) && (dist2 < 0)) return false;
    if ((dist1 >= 0) && (dist2 >= 0)) return true;

    float t = -iOrigin[N]/iDirection[N];
    if (dist1 >= 0) {
      oDist2 = t;
      oP2 = iOrigin + iDirection*t;
    }
    else if (dist2 >= 0) {
      oDist1 = t;
      oP1 = iOrigin + iDirection*t;
    }
    return true;
  }


  template<int N>
  static inline bool clipBoundPlane(const Eigen::Vector3f& iOrigin,
                                    const Eigen::Vector3f& iDirection,
                                    const Eigen::Vector3f& iBound,
                                    Eigen::Vector3f& oP1, Eigen::Vector3f& oP2,
                                    float& oDist1, float& oDist2) {
    float dist1(iBound[N]-oP1[N]), dist2(iBound[N]-oP2[N]);
    if ((dist1 < 0) && (dist2 < 0)) return false;
    if ((dist1 >= 0) && (dist2 >= 0)) return true;

    float t = (iBound[N]-iOrigin[N])/iDirection[N];
    if (dist1 >= 0) {
      oDist2 = t;
      oP2 = iOrigin + iDirection*t;
    }
    else if (dist2 >= 0) {
      oDist1 = t;
      oP1 = iOrigin + iDirection*t;
    }
    return true;
  }


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

  /*
    notes on tsdf
    can smooth each scan with bilateral filter if necessary
    each lidar scan is like a ribbon; subtends sandwich part of double cone
    can march a parallelogram through the volume for each ray
    scan convert at each slice
    can dilate parallelogram
    can measure distance delta along ray and distance laterally/transverse
    how to handle large grid sampling?
    how to handle uncertainty in non-ray directions? (pose error)

    alternative to marching rays: test each voxel in coarse-to-fine manner
      have to affine project in 3d but can do incrementally
      could have luts for distance to rays once projected
   */

protected:
  // TODO: could use aligned memory and striding for efficiency if needed
  Eigen::Vector3i mDimensions;
  Eigen::Affine3f mTransform;
  Eigen::Affine3f mTransformInv;
  std::vector<T> mData;
};

}

#endif
