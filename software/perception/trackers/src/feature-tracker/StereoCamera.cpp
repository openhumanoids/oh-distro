#include "StereoCamera.hpp"

using namespace tracking;

StereoCamera::
StereoCamera() {
}

void StereoCamera::
setLeftCamera(const CameraModel& iCamera) {
  mLeftCamera = iCamera;
}

void StereoCamera::
setRightCamera(const CameraModel& iCamera) {
  mRightCamera = iCamera;
}

const CameraModel& StereoCamera::
getLeftCamera() const {
  return mLeftCamera;
}

const CameraModel& StereoCamera::
getRightCamera() const {
  return mRightCamera;
}

void StereoCamera::
applyPose(const Eigen::Isometry3f& iPose) {
  mLeftCamera.setPose(iPose*mLeftCamera.getPose());
  mRightCamera.setPose(iPose*mRightCamera.getPose());
}
