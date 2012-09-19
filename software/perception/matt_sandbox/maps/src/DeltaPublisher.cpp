#include "DeltaPublisher.hpp"

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_delta_t.hpp>
#include <bot_core/timestamp.h>

#include "MapManager.hpp"
#include "MapChunk.hpp"

DeltaPublisher::
DeltaPublisher() {
  mIsRunning = false;
  mAckSubscription = NULL;
  mNextMessageId = 1;
  setPublishInterval(1000);
  setPublishChannel("MAP_DELTA");
  setAckChannel("MAP_DELTA_ACK");
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
setPublishChannel(const std::string& iChannel) {
  mDeltaChannel = iChannel;
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

    // compute delta from last time
    // TODO: atomic? think about threading / safety
    MapManager::MapDelta delta;
    mManager->computeDelta(delta);

    std::cout << "DeltaPublisher: timer complete" << std::endl;
    if ((delta.mAdded->size() > 0) || (delta.mRemoved->size() > 0)) {

      // create message
      drc::map_delta_t deltaMessage;
      deltaMessage.utime = bot_timestamp_now();
      deltaMessage.message_id = mNextMessageId;
      deltaMessage.map_id = mManager->getActiveMap()->getId();
      deltaMessage.n_added = delta.mAdded->size();
      deltaMessage.added.resize(deltaMessage.n_added);
      for (int i = 0; i < delta.mAdded->size(); ++i) {
        drc::vector_3d_t& pt = deltaMessage.added[i];
        pt.x = delta.mAdded->points[i].x;
        pt.y = delta.mAdded->points[i].y;
        pt.z = delta.mAdded->points[i].z;
      }
      deltaMessage.n_removed = delta.mRemoved->size();
      deltaMessage.removed.resize(deltaMessage.n_removed);
      for (int i = 0; i < delta.mRemoved->size(); ++i) {
        drc::vector_3d_t& pt = deltaMessage.removed[i];
        pt.x = delta.mRemoved->points[i].x;
        pt.y = delta.mRemoved->points[i].y;
        pt.z = delta.mRemoved->points[i].z;
      }
      ++mNextMessageId;

      // publish message
      mLcm->publish(mDeltaChannel, &deltaMessage);
      std::cout << "DeltaPublisher: published message on channel " <<
        mDeltaChannel << std::endl;
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

  // mManager->resetDeltaBase();

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
