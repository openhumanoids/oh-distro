#include "CameraModel.hpp"

CameraModel::
CameraModel() {
  setCalibMatrix(Eigen::Matrix3f::Identity());
  setPose(Eigen::Isometry3f::Identity());
}

void CameraModel::
setCalibMatrix(const Eigen::Matrix3f& iMatrix) {
  mCalib = iMatrix;
  mCalibInv = mCalib.inverse();
}

Eigen::Matrix3f CameraModel::
getCalibMatrix() const {
  return mCalib;
}

void CameraModel::
setPose(const Eigen::Isometry3f& iPose) {
  mPose = iPose;
  mPoseInv = mPose.inverse();
}

Eigen::Isometry3f CameraModel::
getPose() const {
  return mPose;
}

Eigen::Vector3f CameraModel::
pixelToRay(const Eigen::Vector2f& iPixel) {
  return pixelToRay(iPixel[0], iPixel[1]);
}

Eigen::Vector3f CameraModel::
pixelToRay(const Eigen::Vector3f& iPixel) {
  return pixelToRay(iPixel[0], iPixel[1], iPixel[2]);
}

Eigen::Vector3f CameraModel::
pixelToRay(const float iX, const float iY) {
  return pixelToRay(iX, iY, 1);
}

Eigen::Vector3f CameraModel::
pixelToRay(const float iX, const float iY, const float iZ) {
  Eigen::Vector3f pix(iX*iZ, iY*iZ, iZ);
  Eigen::Vector3f ray = mPose*(mCalibInv*pix);
  return ray;
}

Eigen::Vector3f CameraModel::
rayToPixel(const Eigen::Vector3f& iRay) {
  Eigen::Vector3f pix = mCalib*(mPoseInv*iRay);
  pix[0] /= pix[2];
  pix[1] /= pix[2];
  return pix;
}
