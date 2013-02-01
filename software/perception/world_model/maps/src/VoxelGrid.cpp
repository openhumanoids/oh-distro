#include "VoxelGrid.hpp"

using namespace maps;

template<int N>
inline bool clipOriginPlane(const Eigen::Vector3f& iOrigin,
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
inline bool clipBoundPlane(const Eigen::Vector3f& iOrigin,
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


VoxelGrid::
VoxelGrid() {
  setTransform(Eigen::Affine3f::Identity());
  setDimensions(1,1,1);
}

void VoxelGrid::
setDimensions(const int iX, const int iY, const int iZ) {
  mDimensions = Eigen::Vector3i(iX, iY, iZ);
  mRowStrideBytes = iX;
  mPlaneStrideBytes = mRowStrideBytes*iY;
  if (mData.size() != mPlaneStrideBytes*iZ) {
    mData.resize(mPlaneStrideBytes*iZ);
  }
  clear();
}

void VoxelGrid::
setTransform(const Eigen::Affine3f& iTransform) {
  mTransform = iTransform;
  mTransformInv = mTransform.inverse();
}

void VoxelGrid::
clear() {
  std::fill(mData.begin(), mData.end(), 0);
}

bool VoxelGrid::
traceRay(const Eigen::Vector3f& iOrigin,
         const Eigen::Vector3f& iEndPoint,
         std::vector<int64_t>& oIndices) {
  Eigen::Vector3f p1 = mTransformInv*iOrigin;
  Eigen::Vector3f p2 = mTransformInv*iEndPoint;
  Eigen::Vector3f dir = p2 - p1;
  Eigen::Vector3f origin = p1;
  Eigen::Vector3f dims = mDimensions.cast<float>() - Eigen::Vector3f(1,1,1);
  float t1(0), t2(1);

  // plane 1 0 0 0
  if (!clipOriginPlane<0>(origin, dir, p1, p2, t1, t2)) return false;

  // plane 0 1 0 0
  if (!clipOriginPlane<1>(origin, dir, p1, p2, t1, t2)) return false;

  // plane 0 0 1 0
  if (!clipOriginPlane<2>(origin, dir, p1, p2, t1, t2)) return false;

  // plane -1 0 0 xmax
  if (!clipBoundPlane<0>(origin, dir, p1, p2, dims, t1, t2)) return false;

  // plane 0 -1 0 xmax
  if (!clipBoundPlane<1>(origin, dir, p1, p2, dims, t1, t2)) return false;

  // plane 0 0 -1 xmax
  if (!clipBoundPlane<2>(origin, dir, p1, p2, dims, t1, t2)) return false;

  // TODO: 3d bresenham
  // for now, just naive drawing of line from p1 to p2

  // find fastest-changing direction
  Eigen::Vector3f ad = dir.cwiseAbs();
  int maxIdx;
  if ((ad[0] >= ad[1]) && (ad[0] >= ad[2])) maxIdx = 0;
  else if ((ad[1] >= ad[0]) && (ad[1] >= ad[2])) maxIdx = 1;
  else maxIdx = 2;
  Eigen::Vector3f step = dir/ad[maxIdx];

  // TODO: adjust p1 and p2 to lie on integer boundaries
  int numSteps = (p2-p1)[maxIdx];  // TODO: round appropriately
  oIndices.resize(numSteps);
  Eigen::Vector3f p = p1;
  const Eigen::Vector3f half(0.5,0.5,0.5);
  Eigen::Vector3i pInt;
  for (int i = 0; i < numSteps; ++i) {
    p += step;
    pInt = (p+half).cast<int>();
    oIndices[i] = toIndex(pInt[0], pInt[1], pInt[2]);
  }

  return true;
}

