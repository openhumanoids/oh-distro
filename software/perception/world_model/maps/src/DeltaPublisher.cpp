#include "DeltaPublisher.hpp"

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_params_t.hpp>
#include <lcmtypes/drc/map_update_t.hpp>
#include <bot_core/timestamp.h>

#include "MapManager.hpp"
#include "LocalMap.hpp"

DeltaPublisher::
DeltaPublisher() {
  mIsRunning = false;
  mAckSubscription = NULL;
  mNextMessageId = 1;
  setPublishInterval(1000);
  setParamsChannel("MAP_PARAMS");
  setUpdateChannel("MAP_UPDATE");
  setAckChannel("MAP_ACK");
}

DeltaPublisher::
~DeltaPublisher() {
  stop();
}

void DeltaPublisher::
setLcm(boost::shared_ptr<lcm::LCM>& iLcm) {
  mLcm = iLcm;
}

void DeltaPublisher::
setManager(boost::shared_ptr<MapManager>& iManager) {
  mManager = iManager;
}

void DeltaPublisher::
setPublishInterval(const int iMilliseconds) {
  mPublishInterval = iMilliseconds;
}

void DeltaPublisher::
setParamsChannel(const std::string& iChannel) {
  mParamsChannel = iChannel;
}

void DeltaPublisher::
setUpdateChannel(const std::string& iChannel) {
  mUpdateChannel = iChannel;
}

void DeltaPublisher::
setAckChannel(const std::string& iChannel) {
  mAckChannel = iChannel;
}

void DeltaPublisher::
operator()() {

  while (mIsRunning) {
    if ((mManager == NULL) || (mLcm == NULL)) {
      std::cout << "DeltaPublisher: manager or lcm is null; exiting loop" <<
        std::endl;
      break;
    }

    // wait for timer expiry
    boost::asio::io_service service;
    boost::asio::deadline_timer timer(service);
    timer.expires_from_now(boost::posix_time::milliseconds(mPublishInterval));
    timer.wait();
    std::cout << "DeltaPublisher: timer complete" << std::endl;

    // check whether map has been established at other end
    if (mManager->getActiveMap() != NULL) {
      boost::shared_ptr<LocalMap> activeMap = mManager->getActiveMap();
      if (mSentMapIds.find(activeMap->getId()) == mSentMapIds.end()) {

        // create map parameters message
        drc::map_params_t paramsMessage;
        paramsMessage.utime = bot_timestamp_now();
        paramsMessage.message_id = mNextMessageId;
        paramsMessage.map_id = activeMap->getId();
        paramsMessage.resolution = activeMap->getResolution();
        Eigen::Vector3d dims =
          activeMap->getBoundMax() - activeMap->getBoundMin();
        paramsMessage.dimensions[0] = dims[0];
        paramsMessage.dimensions[1] = dims[1];
        paramsMessage.dimensions[2] = dims[2];
        Eigen::Isometry3d xform = activeMap->getTransformToLocal();
        Eigen::Isometry3d::TranslationPart trans = xform.translation();
        paramsMessage.transform_to_local.translation.x = trans[0];
        paramsMessage.transform_to_local.translation.y = trans[1];
        paramsMessage.transform_to_local.translation.z = trans[2];
        Eigen::Quaterniond quat(xform.rotation());
        paramsMessage.transform_to_local.rotation.x = quat.x();
        paramsMessage.transform_to_local.rotation.y = quat.y();
        paramsMessage.transform_to_local.rotation.z = quat.z();
        paramsMessage.transform_to_local.rotation.w = quat.w();
        ++mNextMessageId;

        // send parameters message
        mSentMapIds[paramsMessage.map_id] = false;
        mUnacknowledgedMessageIds[paramsMessage.message_id] =
          paramsMessage.map_id;
        mLcm->publish(mParamsChannel, &paramsMessage);
        std::cout << "DeltaPublisher: published map params on channel " <<
          mParamsChannel << std::endl;
      }
    }

    // compute delta from last time
    // TODO: atomic? think about threading / safety
    MapManager::MapDelta delta;
    mManager->computeDelta(delta);

    if ((delta.mAdded->size() > 0) || (delta.mRemoved->size() > 0)) {

      // create message
      drc::map_update_t updateMessage;
      updateMessage.utime = bot_timestamp_now();
      updateMessage.message_id = mNextMessageId;
      updateMessage.map_id = mManager->getActiveMap()->getId();
      updateMessage.n_added = delta.mAdded->size();
      updateMessage.added.resize(updateMessage.n_added);
      for (int i = 0; i < delta.mAdded->size(); ++i) {
        drc::vector_3d_t& pt = updateMessage.added[i];
        pt.x = delta.mAdded->points[i].x;
        pt.y = delta.mAdded->points[i].y;
        pt.z = delta.mAdded->points[i].z;
      }
      updateMessage.n_removed = delta.mRemoved->size();
      updateMessage.removed.resize(updateMessage.n_removed);
      for (int i = 0; i < delta.mRemoved->size(); ++i) {
        drc::vector_3d_t& pt = updateMessage.removed[i];
        pt.x = delta.mRemoved->points[i].x;
        pt.y = delta.mRemoved->points[i].y;
        pt.z = delta.mRemoved->points[i].z;
      }
      ++mNextMessageId;

      // publish message
      mUnacknowledgedMessageIds[updateMessage.message_id] =
        updateMessage.map_id;
      mLcm->publish(mUpdateChannel, &updateMessage);
      std::cout << "DeltaPublisher: published delta on channel " <<
        mUpdateChannel << std::endl;
      mManager->resetDeltaBase();
    }
  }
}

bool DeltaPublisher::
start() {
  if (mIsRunning) {
    return false;
  }
  mIsRunning = true;
  mAckSubscription =
    mLcm->subscribe(mAckChannel, &DeltaPublisher::onAck, this);
  boost::thread thread(boost::ref(*this));
  // TODO: keep track of thread? join?
  return true;
}

bool DeltaPublisher::
stop() {
  if (!mIsRunning) {
    return false;
  }
  mIsRunning = false;
  mLcm->unsubscribe(mAckSubscription);
  mAckSubscription = NULL;
  return true;
}

void DeltaPublisher::
onAck(const lcm::ReceiveBuffer* iBuf,
      const std::string& iChannel,
      const drc::message_ack_t* iMessage) {
  std::cout << "DeltaPublisher: message receipt acknowledged for " <<
    iMessage->message_id << std::endl;

  // check message off list of unacknowledged and mark map as sent
  std::unordered_map<int64_t,int64_t>::iterator item =
    mUnacknowledgedMessageIds.find(iMessage->message_id);
  if (item != mUnacknowledgedMessageIds.end()) {
    mSentMapIds[item->second] = true;
    mUnacknowledgedMessageIds.erase(item);
  }

  //mManager->resetDeltaBase();

  // TODO
  /* ALGO

     on wakeup, look at current state
     compute delta between base and cur
     add to send queue
     send everything in send queue in order

     on ack, take off send queue

     what if ack message is lost?
     re-send same message again, gets ignored at receiver

     ideally,
     . if couldn't be decoded at receiver, discard and send new delta
         because old message is obe
     . if received but not ack'd, update base and send new delta next time
     . but we don't know which case is true at sender or receiver

     sequential message ids:
     . "first" tag, which will also create map
     . subsequent messages increment id counter by 1
     . keep trying to send a message until it is acknowledged
   */
}
