#include "BotWrapper.hpp"

#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <lcmtypes/bot_param/set_t.hpp>
#include <drc_utils/Clock.hpp>

using namespace maps;

namespace {
  const std::string kSetParamChannel = "PARAM_SET";

  void publishSetParamMessage(const BotWrapper& iBotWrapper,
                              const std::string& iKey,
                              const std::string& iValue,
                              const bool iIsArray=false) {
    BotParam* botParam = iBotWrapper.getBotParam();
    bot_param::set_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.sequence_number = bot_param_get_seqno(botParam);
    msg.server_id = bot_param_get_server_id(botParam);
    msg.numEntries = 1;
    msg.entries.resize(1);
    msg.entries[0].key = iKey;
    msg.entries[0].is_array = iIsArray;
    msg.entries[0].value = iValue;
    iBotWrapper.getLcm()->publish(kSetParamChannel, &msg);
  }
}

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
  if (mBotParam != NULL) {
    mBotFrames = bot_frames_get_global(mLcm->getUnderlyingLCM(), mBotParam);
  }
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

template <typename T>
bool BotWrapper::
getTransform(const std::string& iFrom, const std::string& iTo,
             Eigen::Transform<T,3,Eigen::Isometry>& oTransform,
             const int64_t iTime) const {
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

template <typename T>
bool BotWrapper::
getTransform(const std::string& iFrom, const std::string& iTo,
             Eigen::Quaternion<T>& oRot, Eigen::Matrix<T,3,1>& oTrans,
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
  oRot = Eigen::Quaternion<T>(trans.rot_quat[0], trans.rot_quat[1],
                              trans.rot_quat[2], trans.rot_quat[3]);
  oTrans << trans.trans_vec[0], trans.trans_vec[1], trans.trans_vec[2];
  return true;
}

namespace maps {
  // explicit instantiations
  template bool BotWrapper::
  getTransform(const std::string& iFrom, const std::string& iTo,
               Eigen::Isometry3f& oTransform, const int64_t iTime) const;
  template bool BotWrapper::
  getTransform(const std::string& iFrom, const std::string& iTo,
               Eigen::Isometry3d& oTransform, const int64_t iTime) const;
  template bool BotWrapper::
  getTransform(const std::string& iFrom, const std::string& iTo,
               Eigen::Quaternionf& oRot, Eigen::Vector3f& oTrans,
               const int64_t iTime) const;
  template bool BotWrapper::
  getTransform(const std::string& iFrom, const std::string& iTo,
               Eigen::Quaterniond& oRot, Eigen::Vector3d& oTrans,
               const int64_t iTime) const;
}


// bot param methods

bool BotWrapper::
hasKey(const std::string& iKey) const {
  if (mBotParam == NULL) return false;
  return (0 != bot_param_has_key(mBotParam, iKey.c_str()));
}

bool BotWrapper::
set(const std::string& iKey, const std::string& iValue) {
  if (mBotParam == NULL) return false;
  publishSetParamMessage(*this, iKey, iValue);
  return true;
}

bool BotWrapper::
set(const std::string& iKey, const int iValue) {
  std::ostringstream oss;
  oss << iValue;
  return set(iKey, oss.str());
}

bool BotWrapper::
set(const std::string& iKey, const double iValue) {
  std::ostringstream oss;
  oss << iValue;
  return set(iKey, oss.str());
}

bool BotWrapper::
set(const std::string& iKey, const bool iValue) {
  return set(iKey, iValue ? std::string("true") : std::string("false"));
}

std::string BotWrapper::
get(const std::string& iKey) const {
  std::string val;
  return get(iKey,val) ? val : "";
}

int BotWrapper::
getInt(const std::string& iKey) const {
  int val;
  return get(iKey,val) ? val : -12345678;
}

double BotWrapper::
getDouble(const std::string& iKey) const {
  double val;
  return get(iKey,val) ? val : std::nan("");
}

bool BotWrapper::
getBool(const std::string& iKey) const {
  bool val;
  return get(iKey,val) ? val : false;
}

bool BotWrapper::
get(const std::string& iKey, std::string& oValue) const {
  char* val = NULL;
  if (mBotParam == NULL) return false;
  if (bot_param_get_str(mBotParam, iKey.c_str(), &val) != 0) return false;
  oValue = val;
  return true;
}

bool BotWrapper::
get(const std::string& iKey, int& oValue) const {
  if (mBotParam == NULL) return false;
  if (bot_param_get_int(mBotParam, iKey.c_str(), &oValue) != 0) return false;
  return true;
}

bool BotWrapper::
get(const std::string& iKey, double& oValue) const {
  if (mBotParam == NULL) return false;
  if (bot_param_get_double(mBotParam, iKey.c_str(), &oValue) != 0) return false;
  return true;
}

bool BotWrapper::
get(const std::string& iKey, bool& oValue) const {
  if (mBotParam == NULL) return false;
  int val = 0;
  if (bot_param_get_boolean(mBotParam, iKey.c_str(), &val) != 0) return false;
  oValue = (val!=0);
  return true;
}

std::vector<std::string> BotWrapper::
getKeys(const std::string& iKey) const {
  std::vector<std::string> subkeys;
  if (mBotParam == NULL) return subkeys;
  char** subkeysRaw = bot_param_get_subkeys(mBotParam, iKey.c_str());
  if (subkeysRaw == NULL) return subkeys;
  for (char** subkeyPtr = subkeysRaw; *subkeyPtr != NULL; ++subkeyPtr) {
    subkeys.push_back(std::string(*subkeyPtr));
  }
  bot_param_str_array_free(subkeysRaw);
  return subkeys;
}
