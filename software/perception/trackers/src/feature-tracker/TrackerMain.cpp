#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <list>
#include <unordered_map>
#include <limits>

#include <opencv2/opencv.hpp>
#include <zlib.h>

#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <bot_core/trans.h>

#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/multisense/images_t.hpp>
#include <lcmtypes/drc/tracker_command_t.hpp>
#include <lcmtypes/drc/affordance_plus_collection_t.hpp>

#include "BotUtils.hpp"
#include "StereoCamera.hpp"
#include "FeatureBasedTracker.hpp"

using namespace tracking;

// structure to hold one frame of data to update the tracker
struct UpdateData {
  bot_core::image_t mStereoRaw;
  bot_core::image_t mDisparityRaw;
  cv::Mat mLeftImage;
  cv::Mat mRightImage;
  cv::Mat mDisparity;
  Eigen::Isometry3f mHeadPose;
  bool mReady;
};

// structure to hold information for initializing new track
struct InitializeData {
  int64_t mTimestamp;
  int mObjectId;
  cv::Mat mMask;
  Eigen::Isometry3f mObjectPose;
};

// main structure
struct State {
  std::shared_ptr<lcm::LCM> mLcm;

  BotParam* mBotParam;
  BotFrames* mBotFrames;
  StereoCamera mCamera;

  int mTrackerId;

  size_t mMaxBufferSize;
  std::mutex mBufferMutex;
  std::list<bot_core::image_t> mMaskBuffer;
  std::list<bot_core::image_t> mStereoBuffer;
  std::list<bot_core::image_t> mDisparityBuffer;
  std::list<drc::affordance_t> mAffordanceBuffer;

  UpdateData mUpdateData;
  InitializeData mInitData;

  typedef std::unordered_map<int,drc::affordance_t> AffordanceMap;
  AffordanceMap mLatestAffordances;

  State() {
    // lcm and subscriptions
    mLcm.reset(new lcm::LCM());
    mLcm->subscribe("CAMERALEFT_MASKZIPPED", &State::onMask, this);
    mLcm->subscribe("MULTISENSE_LD", &State::onMultisense, this);
    mLcm->subscribe("CAMERA", &State::onStereo, this);
    mLcm->subscribe("AFFORDANCE_PLUS_COLLECTION", &State::onAffPlus, this);
    mLcm->subscribe("TRACKER_COMMAND", &State::onCommand, this);   

    // configure camera
    mBotParam = bot_param_new_from_server(mLcm->getUnderlyingLCM(), 0);
    BotUtils::configure(mBotParam, mCamera);

    // configure frames
    mBotFrames = bot_frames_new(mLcm->getUnderlyingLCM(), mBotParam);

    // other initializations
    mMaxBufferSize = 30;
    mUpdateData.mReady = false;
    mInitData.mObjectId = -1;
    mTrackerId = 11;
  }

  ~State() {
    bot_frames_destroy(mBotFrames);
    bot_param_destroy(mBotParam);
  }

  template<typename T>
  const T* find(const std::list<T>& iBuffer, const int64_t iTimestamp) const {
    typename std::list<T>::const_iterator iter;
    for (iter = iBuffer.begin(); iter != iBuffer.end(); ++iter) {
      if (iter->utime == iTimestamp) return &(*iter);
    }
    return NULL;
  }

  void checkSynchronized(const int64_t iTimestamp) {
    if (mUpdateData.mReady) return;
    std::list<bot_core::image_t>::iterator iter;
    const bot_core::image_t* stereo = find(mStereoBuffer, iTimestamp);
    const bot_core::image_t* disparity = find(mDisparityBuffer, iTimestamp);
    if ((stereo != NULL) && (disparity != NULL)) {
      mUpdateData.mStereoRaw = *stereo;
      mUpdateData.mDisparityRaw = *disparity;
      mUpdateData.mReady = true;
    }
  }

  void onMask(const lcm::ReceiveBuffer* iBuf, const std::string& iChan,
              const bot_core::image_t* iMessage) {
    std::lock_guard<std::mutex> lock(mBufferMutex);
    mMaskBuffer.push_back(*iMessage);
    if (mMaskBuffer.size() > mMaxBufferSize) mMaskBuffer.pop_front();
    checkSynchronized(iMessage->utime);
  }

  void onMultisense(const lcm::ReceiveBuffer* iBuf, const std::string& iChan,
                    const multisense::images_t* iMessage) {
    //std::cout << "received multisense " << iMessage->utime << std::endl;
    std::lock_guard<std::mutex> lock(mBufferMutex);
    for (int i = 0; i < iMessage->n_images; ++i) {
      if (iMessage->image_types[i] != multisense::images_t::DISPARITY) {
        continue;
      }
      mDisparityBuffer.push_back(iMessage->images[i]);
      if (mDisparityBuffer.size() > mMaxBufferSize) {
        mDisparityBuffer.pop_front();
      }
      break;
    }
    checkSynchronized(iMessage->utime);
  }

  void onStereo(const lcm::ReceiveBuffer* iBuf, const std::string& iChan,
                const bot_core::image_t* iMessage) {
    //std::cout << "received stereo " << iMessage->utime << std::endl;
    std::lock_guard<std::mutex> lock(mBufferMutex);
    mStereoBuffer.push_back(*iMessage);
    if (mStereoBuffer.size() > mMaxBufferSize) mStereoBuffer.pop_front();
    checkSynchronized(iMessage->utime);
  }

  void onAffPlus(const lcm::ReceiveBuffer* iBuf, const std::string& iChan,
                 const drc::affordance_plus_collection_t* iMessage) {
    if (mInitData.mObjectId < 0) return;
    for (size_t i = 0; i < iMessage->affs_plus.size(); ++i) {
      const drc::affordance_t& aff = iMessage->affs_plus[i].aff;

      // create new if this is an init
      if (aff.uid == mInitData.mObjectId) {
        mAffordanceBuffer.push_back(aff);
        mLatestAffordances[aff.uid] = aff;
        if (mAffordanceBuffer.size() > mMaxBufferSize) {
          mAffordanceBuffer.pop_front();
        }
        continue;
      }

      // add to latest affordances if this is an update
      AffordanceMap::const_iterator item = mLatestAffordances.find(aff.uid);
      if (item == mLatestAffordances.end()) continue;
      mLatestAffordances[item->first] = aff;
    }
  }

  void onCommand(const lcm::ReceiveBuffer* iBuf, const std::string& iChan,
                 const drc::tracker_command_t* iMessage) {
    if (iMessage->tracker_id != mTrackerId) return;
    if (iMessage->tracker_type == drc::tracker_command_t::STOP) {
      // TODO: handle this case
      return;
    }
    if (iMessage->tracker_type != drc::tracker_command_t::FEATURE) return;
    mInitData.mObjectId = iMessage->uid;
    mInitData.mTimestamp = iMessage->utime;
  }

};

struct TrackerWrapper {
  std::shared_ptr<State> mState;

  bool mIsRunning;
  std::thread mThread;

  FeatureBasedTracker mTracker;

  TrackerWrapper(std::shared_ptr<State> iState) {
    mState = iState;

    mTracker.setCamera(mState->mCamera);
    mTracker.setPatchRadius(7,7);
    mTracker.setMinMatchScore(0.8);
  }

  void decodeUpdateData() {
    UpdateData& upData = mState->mUpdateData;
    // convert disparity image
    int w = upData.mDisparityRaw.width;
    int h = upData.mDisparityRaw.height;
    void* data = upData.mDisparityRaw.data.data();

    cv::Mat(h, w, CV_16UC1, data).convertTo(upData.mDisparity, CV_32F, 1.0f/16);

    // get pose for stereo pair
    BotTrans trans;
    bot_frames_get_trans_with_utime(mState->mBotFrames, "head", "local",
                                    upData.mStereoRaw.utime, &trans);
    Eigen::Quaternionf q(trans.rot_quat[0], trans.rot_quat[1],
                         trans.rot_quat[2], trans.rot_quat[3]);
    upData.mHeadPose = Eigen::Isometry3f::Identity();
    upData.mHeadPose.linear() = q.matrix();
    upData.mHeadPose.translation() =
      Eigen::Vector3f(trans.trans_vec[0], trans.trans_vec[1],
                      trans.trans_vec[2]);

    // split left and right images
    w = upData.mStereoRaw.width;
    h = upData.mStereoRaw.height/2;
    data = upData.mStereoRaw.data.data();
    cv::Mat img(2*h, w, CV_8UC1, data);
    upData.mLeftImage = img.rowRange(0, h).clone();
    upData.mRightImage = img.rowRange(h, 2*h).clone();
  }

  bool decodeInitData() {
    InitializeData& initData = mState->mInitData;

    // find mask with matching timestamp
    const bot_core::image_t* maskRaw =
      mState->find(mState->mMaskBuffer, mState->mUpdateData.mStereoRaw.utime);
    if (maskRaw == NULL) return false;

    // decode mask
    unsigned long numBytes = maskRaw->width*maskRaw->height;
    std::vector<uint8_t> bytes(numBytes);
    uncompress(bytes.data(), &numBytes, maskRaw->data.data(), maskRaw->size);
    cv::Mat temp(maskRaw->height, maskRaw->width, CV_8UC1, bytes.data());
    cv::Mat1b mask = (temp == (64 + initData.mObjectId));
    initData.mMask = mask;
    // TODO: how to quickly synchronize stereo pair, disparity, mask
    // could generate mask rather than waiting for it
    // could just grab latest mask

    // find closest affordance in time
    std::list<drc::affordance_t>::iterator iter;
    int64_t minTimeDiff = std::numeric_limits<int64_t>::max();
    drc::affordance_t* affordance = NULL;
    for (iter = mState->mAffordanceBuffer.begin();
         iter != mState->mAffordanceBuffer.end(); ++iter) {
      int64_t timeDiff = iter->utime - mState->mUpdateData.mStereoRaw.utime;
      if (abs(timeDiff) < abs(minTimeDiff)) {
        minTimeDiff = timeDiff;
        affordance = &(*iter);
      }
    }
    if (affordance == NULL) return false;

    // populate object pose
    const double* pos = affordance->origin_xyz;
    const double* rpy = affordance->origin_rpy;
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
    Eigen::Isometry3d pose =  Eigen::Isometry3d::Identity();
    pose.linear() = rot;
    pose.translation() << pos[0], pos[1], pos[2];
    initData.mObjectPose = pose.matrix().cast<float>();

    return true;
  }

  void publishAffordance(const int iId, const TrackedObject::State& iState) {
    State::AffordanceMap::const_iterator item =
      mState->mLatestAffordances.find(iId);
    if (item == mState->mLatestAffordances.end()) return;
    drc::affordance_t msg = item->second;
    msg.aff_store_control = drc::affordance_t::UPDATE;
    Eigen::Vector3d rpy =
      iState.mPose.linear().cast<double>().eulerAngles(0,1,2);
    std::swap(rpy[0], rpy[2]);
    Eigen::Vector3d pos = iState.mPose.translation().cast<double>();
    for (int k = 0; k < 3; ++k) {
      msg.origin_rpy[k] = rpy[k];
      msg.origin_xyz[k] = pos[k];
    }
    mState->mLcm->publish("AFFORDANCE_TRACK", &msg);
  }

  void operator()() {
    mIsRunning = true;
    while (mIsRunning) {

      // if there is new unprocessed image data
      if (mState->mUpdateData.mReady) {
        auto startTime = std::chrono::high_resolution_clock::now();
        decodeUpdateData();
        mTracker.setData(mState->mUpdateData.mStereoRaw.utime,
                         mState->mUpdateData.mLeftImage,
                         mState->mUpdateData.mRightImage,
                         mState->mUpdateData.mDisparity,
                         mState->mUpdateData.mHeadPose);
        if (!mTracker.update()) {
          std::cout << "Error: could not update this frame" << std::endl;
        }
        else {
          auto endTime = std::chrono::high_resolution_clock::now();
          auto timeDiff = std::chrono::duration_cast<std::chrono::milliseconds>
            (endTime - startTime);
          std::cout << "Updated in " << timeDiff.count()/1e3 <<
            "s" << std::endl;
          std::vector<int> ids = mTracker.getAllTrackIds();
          for (size_t k = 0; k < ids.size(); ++k) {
            TrackedObject::State state = mTracker.getCurrentState(ids[k]);
            publishAffordance(ids[k], state);
            // TODO: debug info
          }
        }

        if (mState->mInitData.mObjectId >= 0) {

          // try to find mask
          std::cout << "TRYING TO INITIALIZE" << std::endl;
          if (decodeInitData()) {
            if (mTracker.initialize(mState->mInitData.mObjectId,
                                    mState->mInitData.mMask,
                                    mState->mInitData.mObjectPose)) {
              std::cout << "INITIALIZED!" << std::endl;
            }
            else {
              std::cout << "Error: Failed to initialize track" << std::endl;
            }
            mState->mInitData.mObjectId = -1;
          }
        }
        mState->mUpdateData.mReady = false;
      }

      // sleep if we did no work
      else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }
};

int main(const int iArgc, const char** iArgv) {
  // create main object
  std::shared_ptr<State> state(new State());

  // start the processing thread
  TrackerWrapper tracker(state);
  tracker.mThread = std::thread(std::ref(tracker));

  // start lcm loop
  while (0 == state->mLcm->handle()) {}

  // finish up tracker thread
  tracker.mIsRunning = false;
  tracker.mThread.join();

  return 0;
}
