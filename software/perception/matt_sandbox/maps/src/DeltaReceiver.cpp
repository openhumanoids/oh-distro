#include "DeltaReceiver.hpp"

#include <boost/thread.hpp>
#include <lcmtypes/drc/message_ack_t.hpp>
#include <bot_core/timestamp.h>

#include "MapManager.hpp"
#include "MapChunk.hpp"

DeltaReceiver::
DeltaReceiver() {
  mIsRunning = false;
  mDeltaSubscription = NULL;
  setDeltaChannel("MAP_DELTA");
  setAckChannel("MAP_DELTA_ACK");
}

DeltaReceiver::
~DeltaReceiver() {
  stop();
}

void DeltaReceiver::
setManager(boost::shared_ptr<MapManager>& iManager) {
  mManager = iManager;
}

void DeltaReceiver::
setLcm(boost::shared_ptr<lcm::LCM>& iLcm) {
  mLcm = iLcm;
}

void DeltaReceiver::
setDeltaChannel(const std::string& iChannel) {
  mDeltaChannel = iChannel;
}

void DeltaReceiver::
setAckChannel(const std::string& iChannel) {
  mAckChannel = iChannel;
}
  
bool DeltaReceiver::
start() {
  if (mIsRunning) {
    return false;
  }
  mIsRunning = true;

  mDeltaSubscription =
    mLcm->subscribe(mDeltaChannel, &DeltaReceiver::onDelta, this);
  
  boost::thread thread(boost::ref(*this));
  // TODO: keep track of thread? join?

  return true;
}

bool DeltaReceiver::
stop() {
  if (!mIsRunning) {
    return false;
  }
  mLcm->unsubscribe(mDeltaSubscription);
  mDeltaSubscription = NULL;
  mIsRunning = false;
  mDataBuffer.unblock();
  return true;
}

void DeltaReceiver::
operator()() {
  while (mIsRunning) {
    drc::map_delta_t delta;
    if (mDataBuffer.waitForData(delta)) {

      // if this message has not been received, apply delta
      if (!messageReceived(delta.message_id)) {

        // TODO: what coordinate system? compressed leaves?

        // switch to proper map
        if (!mManager->useMap(delta.map_id)) {
          mManager->createMap(Eigen::Isometry3d::Identity(), delta.map_id);
        }

        // add points
        MapChunk::PointCloud::Ptr cloudAdded(new MapChunk::PointCloud());
        cloudAdded->width = delta.added.size();
        cloudAdded->height = 1;
        cloudAdded->is_dense = false;
        cloudAdded->points.resize(delta.added.size());
        for (int i = 0; i < delta.added.size(); ++i) {
          cloudAdded->points[i].x = delta.added[i].x;
          cloudAdded->points[i].y = delta.added[i].y;
          cloudAdded->points[i].z = delta.added[i].z;
        }
        mManager->getActiveMap()->add(cloudAdded,
                                      Eigen::Isometry3d::Identity());

        // remove points
        MapChunk::PointCloud::Ptr cloudRemoved(new MapChunk::PointCloud());
        cloudRemoved->width = delta.removed.size();
        cloudRemoved->height = 1;
        cloudRemoved->is_dense = false;
        cloudRemoved->points.resize(delta.removed.size());
        for (int i = 0; i < delta.removed.size(); ++i) {
          cloudRemoved->points[i].x = delta.removed[i].x;
          cloudRemoved->points[i].y = delta.removed[i].y;
          cloudRemoved->points[i].z = delta.removed[i].z;
        }
        mManager->getActiveMap()->remove(cloudRemoved);

        // note that this message was received
        mReceivedMessages.insert(delta.message_id);
      }

      // acknowledge the message
      drc::message_ack_t ack;
      ack.utime = bot_timestamp_now();
      ack.message_id = delta.message_id;
      mLcm->publish(mAckChannel, &ack);
    }
  }
}

void DeltaReceiver::
onDelta(const lcm::ReceiveBuffer* iBuf,
        const std::string& iChannel,
        const drc::map_delta_t* iMessage) {
  if (!mIsRunning) {
    return;
  }

  std::cout << "DeltaReceiver: received delta (" <<
    iMessage->added.size() << " added, " << iMessage->removed.size() <<
    " removed)" << std::endl;

  mDataBuffer.push(*iMessage);
}

bool DeltaReceiver::
messageReceived(const int64_t iId) {
  return (mReceivedMessages.find(iId) != mReceivedMessages.end());
}
