#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <ConciseArgs>
#include <lcm/lcm-cpp.hpp>

#include <affordance/AffordanceUpWrapper.h>

#include <lcmtypes/bot_core/rigid_transform_t.hpp>
#include <lcmtypes/drc/affordance_t.hpp>
#include <lcmtypes/drc/map_registration_command_t.hpp>

struct AffordanceMetadata {
  int64_t mUpdateTime;
  bool mUserUpdateReceived;
};

struct State {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<drc::BotWrapper> mBotWrapper;
  std::shared_ptr<affordance::AffordanceUpWrapper> mAffordanceWrapper;
  std::unordered_map<int32_t,AffordanceMetadata> mAffordanceMetadata;

  bool mIsRunning;
  std::thread mThread;
  std::condition_variable mCondition;
  std::mutex mMutex;
  int64_t mUpdateTime;
  int64_t mPrevUpdateTime;
  bool mUpdateOnce;
  bool mShouldUpdate;

  State() {
    // initialize some variables
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    mBotWrapper.reset(new drc::BotWrapper(mLcm));
    mAffordanceWrapper.reset
      (new affordance::AffordanceUpWrapper(mLcmWrapper->getBoost()));

    mUpdateTime = mPrevUpdateTime = 0;
    mIsRunning = false;
    mUpdateOnce = false;
    mShouldUpdate = false;

    mLcm->subscribe("MAP_LOCAL_CORRECTION", &State::onCorrection, this);
    mLcm->subscribe("MAP_REGISTRATION_COMMAND", &State::onCommand, this);
    mLcm->subscribe(affordance::AffordanceServer::AFFORDANCE_TRACK_CHANNEL,
                    &State::onAffordanceTrack, this);
  }

  ~State() {
    stop();
  }

  void onAffordanceTrack(const lcm::ReceiveBuffer* iBuf,
                         const std::string& iChannel,
                         const drc::affordance_t* iMessage) {
    AffordanceMetadata meta;
    meta.mUserUpdateReceived = true;
    meta.mUpdateTime = iMessage->utime;
    if (mAffordanceMetadata.find(iMessage->uid)==mAffordanceMetadata.end()) {
      mAffordanceMetadata[iMessage->uid] = meta;
      return;
    }
    if (mAffordanceMetadata[iMessage->uid].mUpdateTime == iMessage->utime) {
      return;
    }
    mAffordanceMetadata[iMessage->uid] = meta;
  }

  void onCorrection(const lcm::ReceiveBuffer* iBuf,
                    const std::string& iChannel,
                    const bot_core::rigid_transform_t* iMessage) {
    mUpdateTime = iMessage->utime;
    mCondition.notify_one();
  }

  void onCommand(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const drc::map_registration_command_t* iMessage) {
    switch (iMessage->command) {
    case drc::map_registration_command_t::AFF_UPDATE_START:
      mShouldUpdate = true;
      break;
    case drc::map_registration_command_t::AFF_UPDATE_PAUSE:
      mShouldUpdate = false;
      break;
    case drc::map_registration_command_t::AFF_UPDATE_FORCE:
      mUpdateOnce = true;
      mShouldUpdate = true;
      break;
    }
  }

  void operator()() {
    mIsRunning = true;
    while (mIsRunning) {
      std::unique_lock<std::mutex> lock(mMutex);
      mCondition.wait_for(lock,std::chrono::milliseconds(100));
      if (mUpdateTime != mPrevUpdateTime) {
        Eigen::Isometry3d xformCur;
        if (!mBotWrapper->getTransform("corrected_local", "local",
                                       xformCur, mUpdateTime)) continue;
        std::vector<affordance::AffConstPtr> affordances;
        mAffordanceWrapper->getAllAffordances(affordances);
        for (auto aff : affordances) {

          // if we haven't seen this affordance before, do not update
          // but keep track of the time when we first saw it
          if (mAffordanceMetadata.find(aff->_uid)==mAffordanceMetadata.end()) {
            AffordanceMetadata meta;
            meta.mUpdateTime = mUpdateTime;
            meta.mUserUpdateReceived = false;
            mAffordanceMetadata[aff->_uid] = meta;
            continue;
          }
          
          if (!mShouldUpdate) continue;

          // check to see if the user updated this affordance
          AffordanceMetadata& meta = mAffordanceMetadata[aff->_uid];
          if (meta.mUserUpdateReceived) {
            if (mUpdateTime >= meta.mUpdateTime) {
              meta.mUpdateTime = mUpdateTime;
              meta.mUserUpdateReceived = false;
            }
            continue;
          }

          // get transform at time of last affordance update
          int64_t affUpdateTime = meta.mUpdateTime;
          Eigen::Isometry3d xformAff;
          if (!mBotWrapper->getTransform("corrected_local", "local",
                                         xformAff, affUpdateTime)) continue;
          Eigen::Isometry3d xformUpdate = xformCur.inverse()*xformAff;

          // create message
          drc::affordance_t msg;
          aff->toMsg(&msg);

          // update affordance pose
          Eigen::Isometry3d curAffPose = Eigen::Isometry3d::Identity();
          curAffPose.translation() <<
            aff->_origin_xyz[0], aff->_origin_xyz[1], aff->_origin_xyz[2];
          Eigen::Matrix3d rotation;
          rotation =
            Eigen::AngleAxisd(aff->_origin_rpy[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(aff->_origin_rpy[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(aff->_origin_rpy[0], Eigen::Vector3d::UnitX());
          curAffPose.linear() = rotation;
          curAffPose = xformUpdate*curAffPose;

          // extract xyz and rpy from new pose matrix
          for (int k = 0; k < 3; ++k) msg.origin_xyz[k] = curAffPose(k,3);
          Eigen::Vector3d rpy = curAffPose.linear().eulerAngles(2,1,0);
          for (int k = 0; k < 3; ++k) msg.origin_rpy[k] = rpy[2-k];

          // publish update message
          msg.aff_store_control = drc::affordance_t::UPDATE;
          msg.utime = mUpdateTime;
          mLcm->publish(affordance::AffordanceServer::AFFORDANCE_TRACK_CHANNEL,
                        &msg);
          meta.mUpdateTime = mUpdateTime;
          std::cout << "updated affordance " << aff->_uid << std::endl;
        }

        mPrevUpdateTime = mUpdateTime;
        if (mUpdateOnce) {
          mUpdateOnce = false;
          mShouldUpdate = false;
        }
      }
    }
  }

  void start() {
    mThread = std::thread(std::ref(*this));
    mLcmWrapper->startHandleThread(true);
  }

  void stop() {
    mIsRunning = false;
    mLcmWrapper->stopHandleThread();
    if (mThread.joinable()) mThread.join();
    mAffordanceMetadata.clear();
    mUpdateTime = mPrevUpdateTime = 0;
  }
};


int main(const int iArgc, const char** iArgv) {

  // instantiate state object
  State state;

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv);
  // TODO: args
  opt.parse();

  state.start();

  return 0;
}
