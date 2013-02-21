#include "BotFramesWrapper.hpp"

#include <lcm/lcm-cpp.hpp>
#include <bot_frames/bot_frames.h>

using namespace maps;

struct BotFramesWrapper::BotStructures {
  BotParam* mParam;
  BotFrames* mFrames;
};

BotFramesWrapper::
BotFramesWrapper(const BotParam* iParam) {
  mBotStructures.reset(new BotStructures());
  mBotStructures->mParam = (BotParam*)iParam;
}

BotFramesWrapper::
~BotFramesWrapper() {
}

void BotFramesWrapper::
setLcm(const boost::shared_ptr<lcm::LCM>& iLcm) {
  mLcm = iLcm;
  lcm_t* lcm = mLcm->getUnderlyingLCM();
  if (mBotStructures->mParam == NULL) {
    mBotStructures->mParam = bot_param_get_global(lcm, 0);
  }
  mBotStructures->mFrames = bot_frames_get_global(lcm, mBotStructures->mParam);
}

bool BotFramesWrapper::
getTransform(const std::string& iFrom, const std::string& iTo,
             const int64_t iTimestamp, Eigen::Isometry3d& oTransform) {  
  double mat[16];
  if (0 == bot_frames_get_trans_mat_4x4_with_utime(mBotStructures->mFrames,
                                                   iFrom.c_str(), iTo.c_str(),
                                                   iTimestamp, mat)) {
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oTransform(i,j) = mat[i*4+j];
    }
  }
  return true;
}

bool BotFramesWrapper::
getTransform(const std::string& iFrom, const std::string& iTo,
             const int64_t iTimestamp, Eigen::Isometry3f& oTransform) {
  
  double mat[16];
  if (0 == bot_frames_get_trans_mat_4x4_with_utime(mBotStructures->mFrames,
                                                   iFrom.c_str(), iTo.c_str(),
                                                   iTimestamp, mat)) {
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oTransform(i,j) = mat[i*4+j];
    }
  }
  return true;
}

bool BotFramesWrapper::
getTransform(const std::string& iFrom, const std::string& iTo,
             const int64_t iTimestamp,
             Eigen::Vector3d& oTrans, Eigen::Quaterniond& oRot) {
  BotTrans trans;
  if (0 == bot_frames_get_trans_with_utime(mBotStructures->mFrames,
                                           iFrom.c_str(), iTo.c_str(),
                                           iTimestamp, &trans)) {
    return false;
  }
  oTrans = Eigen::Vector3d(trans.trans_vec[0], trans.trans_vec[1],
                           trans.trans_vec[2]);
  oRot = Eigen::Quaterniond(trans.rot_quat[0], trans.rot_quat[1],
                            trans.rot_quat[2], trans.rot_quat[3]);
  return true;
}

bool BotFramesWrapper::
getTransform(const std::string& iFrom, const std::string& iTo,
             const int64_t iTimestamp,
             Eigen::Vector3f& oTrans, Eigen::Quaternionf& oRot) {
  BotTrans trans;
  if (0 == bot_frames_get_trans_with_utime(mBotStructures->mFrames,
                                           iFrom.c_str(), iTo.c_str(),
                                           iTimestamp, &trans)) {
    return false;
  }
  oTrans = Eigen::Vector3f(trans.trans_vec[0], trans.trans_vec[1],
                           trans.trans_vec[2]);
  oRot = Eigen::Quaternionf(trans.rot_quat[0], trans.rot_quat[1],
                            trans.rot_quat[2], trans.rot_quat[3]);
  return true;
}
