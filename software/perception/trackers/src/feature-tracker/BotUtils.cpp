#include "BotUtils.hpp"

#include "StereoCamera.hpp"

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_core/camtrans.h>

using namespace tracking;

static bool
populateCamera(CameraModel& oCamera, const BotParam* iParam,
               const std::string& iName) {
  BotParam* param = const_cast<BotParam*>(iParam);
  BotCamTrans* camTrans = bot_param_get_new_camtrans(param, iName.c_str());
  if (camTrans == NULL) return false;

  // set up intrinsic parameters
  double k00 = bot_camtrans_get_focal_length_x(camTrans);
  double k11 = bot_camtrans_get_focal_length_y(camTrans);
  double k01 = bot_camtrans_get_skew(camTrans);
  double k02 = bot_camtrans_get_principal_x(camTrans);
  double k12 = bot_camtrans_get_principal_y(camTrans);
  Eigen::Matrix3f calibMatrix = Eigen::Matrix3f::Identity();
  calibMatrix << k00,k01,k02, 0,k11,k12, 0,0,1;
  oCamera.setCalibMatrix(calibMatrix);
  bot_camtrans_destroy(camTrans);

  // set up pose wrt head
  char* camFrame = bot_param_get_camera_coord_frame(param, iName.c_str());
  if (camFrame == NULL) return false;
  std::string key = "coordinate_frames." + std::string(camFrame) +
    ".initial_transform";
  free (camFrame);
  BotTrans trans;
  if (0 != bot_param_get_trans(param, key.c_str(), &trans)) return false;
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.translation() = Eigen::Vector3f(trans.trans_vec[0], trans.trans_vec[1],
                                       trans.trans_vec[2]);
  Eigen::Quaternionf quat(trans.rot_quat[0], trans.rot_quat[1],
                          trans.rot_quat[2], trans.rot_quat[3]);
  pose.linear() = quat.matrix();
  oCamera.setPose(pose);

  return true;
}

bool BotUtils::
configure(const BotParam* iParam, StereoCamera& oCamera) {
  if (iParam == NULL) return false;

  CameraModel cameraLeft, cameraRight;
  if (!populateCamera(cameraLeft, iParam, "CAMERALEFT")) return false;
  if (!populateCamera(cameraRight, iParam, "CAMERARIGHT")) return false;
  oCamera.setLeftCamera(cameraLeft);
  oCamera.setRightCamera(cameraRight);

  return true;
}
