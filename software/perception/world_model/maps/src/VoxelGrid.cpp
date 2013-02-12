#include "VoxelGrid.hpp"

using namespace maps;


template <typename T>
VoxelGrid<T>::
VoxelGrid() {
  setTransform(Eigen::Affine3f::Identity());
  setDimensions(1,1,1);
}

template <typename T>
void VoxelGrid<T>::
setDimensions(const int iX, const int iY, const int iZ) {
  mDimensions = Eigen::Vector3i(iX, iY, iZ);
  int totalSize = iX*iY*iZ;
  mData.resize(totalSize);
  clear();
}

template <typename T>
void VoxelGrid<T>::
setTransform(const Eigen::Affine3f& iTransform) {
  mTransform = iTransform;
  mTransformInv = mTransform.inverse();
}

template <typename T>
void VoxelGrid<T>::
clear(const T& iValue) {
  std::fill(mData.begin(), mData.end(), iValue);
}

// TODO: RENAME
/*
template <typename T>
bool VoxelGrid<T>::
isBoxInsideUnitCone(const Eigen::Vector3i& iP1, const Eigen::Vector3i& iP2,
                    const Eigen::Affine3f& iTransform) {
  // TODO: integer math
  std::vector<Eigen::Vector3f> pts(8);
  pts[0] = Eigen::Vector3f(iP1[0], iP1[1], iP1[2]);
  pts[1] = Eigen::Vector3f(iP1[0], iP1[1], iP2[2]);
  pts[2] = Eigen::Vector3f(iP1[0], iP2[1], iP1[2]);
  pts[3] = Eigen::Vector3f(iP1[0], iP2[1], iP2[2]);
  pts[4] = Eigen::Vector3f(iP2[0], iP1[1], iP1[2]);
  pts[5] = Eigen::Vector3f(iP2[0], iP1[1], iP2[2]);
  pts[6] = Eigen::Vector3f(iP2[0], iP2[1], iP1[2]);
  pts[7] = Eigen::Vector3f(iP2[0], iP2[1], iP2[2]);
  int numAbove(0), numBelow(0);
  for (int i = 0; i < 8; ++i) {
    Eigen::Vector3f p = iTransform*pts[i];
    float r2 = p[0]*p[0] + p[1]*p[1];
    float z2 = p[2]*p[2];
    numAbove += (r2>z2);
    numBelow += (r2<-z2);
  }
  return ((numAbove == 8) || (numBelow == 8));
}
*/

// explicit instantiations
namespace maps {
  template class VoxelGrid<int8_t>;
  template class VoxelGrid<int16_t>;
  template class VoxelGrid<int32_t>;
}
