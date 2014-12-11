#include "Camera.hpp"

#include <bot_core/camtrans.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include <iostream>

using namespace drc;

struct Camera::Helper {
  Eigen::Matrix3d mCalibrationMatrix;
  BotCamTrans* mCamTrans;
  bool mShouldFreeCamTrans;

  Helper() {
    mCalibrationMatrix = Eigen::Matrix3d::Identity();
    mCamTrans = NULL;
  }
  ~Helper() {
    if (mShouldFreeCamTrans && (mCamTrans != NULL)) {
      bot_camtrans_destroy(mCamTrans);
    }
  }
};

Camera::
Camera(BotParam* iBotParam, const std::string& iName) {
  mHelper.reset(new Helper());
  mHelper->mCamTrans = bot_param_get_new_camtrans(iBotParam, iName.c_str());
  if (mHelper->mCamTrans == NULL) {
    std::cout << "error: cannot create camera " << iName << std::endl;
  }
  mHelper->mShouldFreeCamTrans = true;
}

Camera::
Camera(BotCamTrans* iCamTrans, const bool iShouldFree) {
  mHelper.reset(new Helper());
  mHelper->mCamTrans = iCamTrans;
  mHelper->mShouldFreeCamTrans = iShouldFree;
}

Camera::
~Camera() {
}

Eigen::Matrix3d Camera::
getCalibrationMatrix() const {
  return mHelper->mCalibrationMatrix;
}

Eigen::Vector2d Camera::
project(const Eigen::Vector3d& iPoint) const {
  double pt[] = {iPoint[0], iPoint[1], iPoint[2]};
  double pix[3];
  bot_camtrans_project_point(mHelper->mCamTrans, pt, pix);
  return Eigen::Vector2d(pix[0], pix[1]);
}

Eigen::Vector3d Camera::
unproject(const Eigen::Vector2d& iPixel) const {
  double ray[3];
  bot_camtrans_unproject_pixel(mHelper->mCamTrans, iPixel[0], iPixel[1], ray);
  return Eigen::Vector3d(ray[0], ray[1], ray[2]);
}
