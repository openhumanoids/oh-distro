#include "DenseGrid3d.hpp"

DenseGrid3d::
DenseGrid3d() {
  setToLocal(Eigen::Affine3d::Identity());
  setDimensions(1,1,1);
}

void DenseGrid3d::
setDimensions(const int iX, const int iY, const int iZ) {
  mDimensions = Eigen::Vector3i(iX, iY, iZ);
  mRowStrideBytes = iX;
  mPlaneStrideBytes = mRowStrideBytes*iY;
  if (mData.size() != mPlaneStrideBytes*iZ) {
    mData.resize(mPlaneStrideBytes*iZ);
  }
  clear();
}

void DenseGrid3d::
setToLocal(const Eigen::Affine3d& iTransform) {
  mToLocalTransform = iTransform;
}

void DenseGrid3d::
clear() {
  std::fill(mData.begin(), mData.end(), 0);
}
