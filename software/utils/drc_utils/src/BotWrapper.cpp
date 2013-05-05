#include "BotWrapper.hpp"

#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

using namespace drc;

BotWrapper::
BotWrapper() {
  setDefaults();
}

BotWrapper::
BotWrapper(const std::shared_ptr<lcm::LCM>& iLcm,
           const BotParam* iBotParam, const BotFrames* iBotFrames) {
  set(iLcm, iBotParam, iBotFrames);
}

BotWrapper::
BotWrapper(const lcm_t* iLcm,
           const BotParam* iBotParam, const BotFrames* iBotFrames) {
  set(iLcm, iBotParam, iBotFrames);
}

void BotWrapper::
setDefaults() {
  mLcm.reset(new lcm::LCM());
  mBotParam = bot_param_get_global(mLcm->getUnderlyingLCM(),0);
  mBotFrames = bot_frames_get_global(mLcm->getUnderlyingLCM(), mBotParam);
}

void BotWrapper::
set(const std::shared_ptr<lcm::LCM>& iLcm,
    const BotParam* iBotParam, const BotFrames* iBotFrames) {
  mLcm = iLcm;
  mBotParam = const_cast<BotParam*>(iBotParam);
  if ((mBotParam == NULL) && (mLcm != NULL)) {
    mBotParam = bot_param_get_global(mLcm->getUnderlyingLCM(),0);
  }
  mBotFrames = const_cast<BotFrames*>(iBotFrames);
  if ((mBotFrames == NULL) && (mLcm != NULL) && (mBotParam != NULL)) {
    mBotFrames = bot_frames_get_global(mLcm->getUnderlyingLCM(), mBotParam);
  }
}

void BotWrapper::
set(const lcm_t* iLcm,
    const BotParam* iBotParam, const BotFrames* iBotFrames) {
  mLcm.reset(new lcm::LCM(const_cast<lcm_t*>(iLcm)));
  mBotParam = const_cast<BotParam*>(iBotParam);
  mBotFrames = const_cast<BotFrames*>(iBotFrames);
  if ((mBotParam == NULL) && (mLcm != NULL)) {
    mBotParam = bot_param_get_global(mLcm->getUnderlyingLCM(),0);
  }
  if ((mBotFrames == NULL) && (mLcm != NULL) && (mBotParam != NULL)) {
    mBotFrames = bot_frames_get_global(mLcm->getUnderlyingLCM(), mBotParam);
  }
}

std::shared_ptr<lcm::LCM> BotWrapper::
getLcm() const {
  return mLcm;
}

BotParam* BotWrapper::
getBotParam() const {
  return mBotParam;
}

BotFrames* BotWrapper::
getBotFrames() const {
  return mBotFrames;
}

int64_t BotWrapper::
getLatestTime(const std::string& iFrom, const std::string& iTo) const {
  if (mBotFrames == NULL) return -1;
  int64_t latestTime;
  bot_frames_get_trans_latest_timestamp
    (mBotFrames, iFrom.c_str(), iTo.c_str(), &latestTime);
  return latestTime;
}

bool BotWrapper::
getTransform(const std::string& iFrom, const std::string& iTo,
             Eigen::Isometry3f& oTransform, const int64_t iTime) const {
  if (mBotFrames == NULL) return false;
  double mat[16];
  if (iTime < 0) {
    if (0 == bot_frames_get_trans_mat_4x4
        (mBotFrames, iFrom.c_str(), iTo.c_str(), mat)) {
      return false;
    }
  }
  else {
    if (0 == bot_frames_get_trans_mat_4x4_with_utime
        (mBotFrames, iFrom.c_str(), iTo.c_str(), iTime, mat)) {
      return false;
    }
  }
  for (int i = 0, idx = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j, ++idx) {
      oTransform(i,j) = mat[idx];
    }
  }
  return true;
}

bool BotWrapper::
getTransform(const std::string& iFrom, const std::string& iTo,
             Eigen::Quaternionf& oRot, Eigen::Vector3f& oTrans,
             const int64_t iTime) const {
  if (mBotFrames == NULL) return false;
  BotTrans trans;
  if (iTime < 0) {;
    if (0 == bot_frames_get_trans
        (mBotFrames, iFrom.c_str(), iTo.c_str(), &trans)) {
      return false;
    }
  }
  else {
    if (0 == bot_frames_get_trans_with_utime
        (mBotFrames, iFrom.c_str(), iTo.c_str(), iTime, &trans)) {
      return false;
    }
  }
  oRot = Eigen::Quaternionf(trans.rot_quat[0], trans.rot_quat[1],
                            trans.rot_quat[2], trans.rot_quat[3]);
  oTrans = Eigen::Vector3f(trans.trans_vec[0], trans.trans_vec[1],
                           trans.trans_vec[2]);
  return true;
}

bool BotWrapper::
getTransform(const std::string& iFrom, const std::string& iTo,
             Eigen::Isometry3d& oTransform, const int64_t iTime) const {
  if (mBotFrames == NULL) return false;
  double mat[16];
  if (iTime < 0) {
    if (0 == bot_frames_get_trans_mat_4x4
        (mBotFrames, iFrom.c_str(), iTo.c_str(), mat)) {
      return false;
    }
  }
  else {
    if (0 == bot_frames_get_trans_mat_4x4_with_utime
        (mBotFrames, iFrom.c_str(), iTo.c_str(), iTime, mat)) {
      return false;
    }
  }
  for (int i = 0, idx = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j, ++idx) {
      oTransform(i,j) = mat[idx];
    }
  }
  return true;
}

bool BotWrapper::
getTransform(const std::string& iFrom, const std::string& iTo,
             Eigen::Quaterniond& oRot, Eigen::Vector3d& oTrans,
             const int64_t iTime) const {
  if (mBotFrames == NULL) return false;
  BotTrans trans;
  if (iTime < 0) {;
    if (0 == bot_frames_get_trans
        (mBotFrames, iFrom.c_str(), iTo.c_str(), &trans)) {
      return false;
    }
  }
  else {
    if (0 == bot_frames_get_trans_with_utime
        (mBotFrames, iFrom.c_str(), iTo.c_str(), iTime, &trans)) {
      return false;
    }
  }
  oRot = Eigen::Quaterniond(trans.rot_quat[0], trans.rot_quat[1],
                            trans.rot_quat[2], trans.rot_quat[3]);
  oTrans = Eigen::Vector3d(trans.trans_vec[0], trans.trans_vec[1],
                           trans.trans_vec[2]);
  return true;
}
