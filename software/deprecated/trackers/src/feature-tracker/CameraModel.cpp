#include "CameraModel.hpp"

using namespace tracking;

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
pixelToRay(const Eigen::Vector2f& iPixel) const {
  return pixelToRay(iPixel[0], iPixel[1]);
}

Eigen::Vector3f CameraModel::
pixelToRay(const Eigen::Vector3f& iPixel) const {
  return pixelToRay(iPixel[0], iPixel[1], iPixel[2]);
}

Eigen::Vector3f CameraModel::
pixelToRay(const float iX, const float iY) const {
  return pixelToRay(iX, iY, 1);
}

Eigen::Vector3f CameraModel::
pixelToRay(const float iX, const float iY, const float iZ) const {
  Eigen::Vector3f pix(iX*iZ, iY*iZ, iZ);
  Eigen::Vector3f ray = mPose.linear()*(mCalibInv*pix);
  return ray;
}

Eigen::Vector3f CameraModel::
pixelToPoint(const Eigen::Vector2f& iPixel) const {
  return pixelToPoint(iPixel[0], iPixel[1]);
}

Eigen::Vector3f CameraModel::
pixelToPoint(const Eigen::Vector3f& iPixel) const {
  return pixelToPoint(iPixel[0], iPixel[1], iPixel[2]);
}

Eigen::Vector3f CameraModel::
pixelToPoint(const float iX, const float iY) const {
  return pixelToPoint(iX, iY, 1);
}

Eigen::Vector3f CameraModel::
pixelToPoint(const float iX, const float iY, const float iZ) const {
  Eigen::Vector3f pix(iX*iZ, iY*iZ, iZ);
  Eigen::Vector3f pt = mPose*(mCalibInv*pix);
  return pt;
}

Eigen::Vector3f CameraModel::
rayToPixel(const Eigen::Vector3f& iRay) const {
  Eigen::Vector3f pix = mCalib*(mPoseInv.linear()*iRay);
  pix[0] /= pix[2];
  pix[1] /= pix[2];
  return pix;
}

Eigen::Vector3f CameraModel::
pointToPixel(const Eigen::Vector3f& iPoint) const {
  Eigen::Vector3f pix = mCalib*(mPoseInv*iPoint);
  pix[0] /= pix[2];
  pix[1] /= pix[2];
  return pix;
}

CameraModel CameraModel::
scaleImage(const float iScale) const {
  Eigen::Matrix3f scaleMatrix;
  scaleMatrix << iScale,0,0, 0,iScale,0, 0,0,1;
  CameraModel cam = *this;
  cam.setCalibMatrix(scaleMatrix*cam.mCalib);
  return cam;
}
