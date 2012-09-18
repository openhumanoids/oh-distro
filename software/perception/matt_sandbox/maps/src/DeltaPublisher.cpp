#include "DeltaPublisher.hpp"

// TODO: maybe make this class manage the cur and prev maps?
// make delta computation atomic?

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
  mNextMessageId = 1;
  setPublishInterval(1000);
  setPublishChannel("MAP_DELTA");
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
  mChannel = iChannel;
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
      mLcm->publish(mChannel, &deltaMessage);
      std::cout << "DeltaPublisher: published message on channel " <<
        mChannel << std::endl;
      mManager->resetDelta();
    }
  }
}

bool DeltaPublisher::
start() {
  if (mIsRunning) {
    return false;
  }
  mIsRunning = true;
  boost::thread thread(*this);
  // TODO: keep track of thread? join?
  return true;
}

bool DeltaPublisher::
stop() {
  if (!mIsRunning) {
    return false;
  }
  mIsRunning = false;
  return true;
}
