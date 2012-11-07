#include "DeltaReceiver.hpp"

#include <boost/thread.hpp>
#include <lcmtypes/drc/message_ack_t.hpp>
#include <bot_core/timestamp.h>

#include "MapManager.hpp"
#include "LocalMap.hpp"

DeltaReceiver::
DeltaReceiver() {
  mIsRunning = false;
  mUpdateSubscription = NULL;
  setParamsChannel("MAP_PARAMS");
  setUpdateChannel("MAP_UPDATE");
  setAckChannel("MAP_ACK");
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
setParamsChannel(const std::string& iChannel) {
  mParamsChannel = iChannel;
}

void DeltaReceiver::
setUpdateChannel(const std::string& iChannel) {
  mUpdateChannel = iChannel;
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

  mParamsSubscription =
    mLcm->subscribe(mParamsChannel, &DeltaReceiver::onParams, this);

  mUpdateSubscription =
    mLcm->subscribe(mUpdateChannel, &DeltaReceiver::onUpdate, this);
  
  boost::thread thread(boost::ref(*this));
  // TODO: keep track of thread? join?

  return true;
}

bool DeltaReceiver::
stop() {
  if (!mIsRunning) {
    return false;
  }
  mLcm->unsubscribe(mUpdateSubscription);
  mUpdateSubscription = NULL;
  mIsRunning = false;
  mDataBuffer.unblock();
  return true;
}

void DeltaReceiver::
operator()() {
  while (mIsRunning) {
    drc::map_update_t delta;
    if (mDataBuffer.waitForData(delta)) {

      // if this message has not been received, apply delta
      if (!messageReceived(delta.message_id)) {

        // switch to proper map
        if (!mManager->useMap(delta.map_id)) {
          std::cout << "DeltaReceiver: error - no map with id " <<
            delta.map_id << " exists yet" << std::endl;
          // TODO mManager->createMap(Eigen::Isometry3d::Identity(), delta.map_id);
        }

        // collect points to add
        LocalMap::PointCloud::Ptr cloudAdded(new LocalMap::PointCloud());
        cloudAdded->width = delta.added.size();
        cloudAdded->height = 1;
        cloudAdded->is_dense = false;
        cloudAdded->points.resize(delta.added.size());
        for (int i = 0; i < delta.added.size(); ++i) {
          cloudAdded->points[i].x = delta.added[i].x;
          cloudAdded->points[i].y = delta.added[i].y;
          cloudAdded->points[i].z = delta.added[i].z;
        }

        // collect points to remove
        LocalMap::PointCloud::Ptr cloudRemoved(new LocalMap::PointCloud());
        cloudRemoved->width = delta.removed.size();
        cloudRemoved->height = 1;
        cloudRemoved->is_dense = false;
        cloudRemoved->points.resize(delta.removed.size());
        for (int i = 0; i < delta.removed.size(); ++i) {
          cloudRemoved->points[i].x = delta.removed[i].x;
          cloudRemoved->points[i].y = delta.removed[i].y;
          cloudRemoved->points[i].z = delta.removed[i].z;
        }

        // add and remove points
        mManager->getActiveMap()->applyChanges(cloudAdded, cloudRemoved);

        // update state id
        // TODO: can set based on delta messages and protocol
        int64_t id = mManager->getActiveMap()->getStateId();
        mManager->getActiveMap()->setStateId(id+1);

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
onParams(const lcm::ReceiveBuffer* iBuf,
         const std::string& iChannel,
         const drc::map_params_t* iMessage) {
  if (!mIsRunning) {
    return;
  }

  std::cout << "DeltaReceiver: received params for map " <<
    iMessage->map_id << std::endl;

  // add new map if it does not exist
  if (!mManager->hasMap(iMessage->map_id)) {
    mManager->setMapResolution(iMessage->resolution);
    Eigen::Vector3d dims;
    dims[0] = iMessage->dimensions[0];
    dims[1] = iMessage->dimensions[1];
    dims[2] = iMessage->dimensions[2];
    mManager->setMapDimensions(dims);
    Eigen::Isometry3d xform = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond quat;
    quat.x() = iMessage->transform_to_local.rotation.x;
    quat.y() = iMessage->transform_to_local.rotation.y;
    quat.z() = iMessage->transform_to_local.rotation.z;
    quat.w() = iMessage->transform_to_local.rotation.w;
    Eigen::Vector3d trans;
    trans[0] = iMessage->transform_to_local.translation.x;
    trans[1] = iMessage->transform_to_local.translation.y;
    trans[2] = iMessage->transform_to_local.translation.z;
    xform.rotate(quat);
    xform.translate(trans);
    mManager->createMap(xform, iMessage->map_id);
    mManager->getActiveMap()->setStateId(0);
  }

  // acknowledge that we received the map parameters
  drc::message_ack_t ack;
  ack.utime = bot_timestamp_now();
  ack.message_id = iMessage->message_id;
  mLcm->publish(mAckChannel, &ack);
}

void DeltaReceiver::
onUpdate(const lcm::ReceiveBuffer* iBuf,
         const std::string& iChannel,
         const drc::map_update_t* iMessage) {
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
