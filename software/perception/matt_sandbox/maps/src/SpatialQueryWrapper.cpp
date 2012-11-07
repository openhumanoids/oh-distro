#include "SpatialQueryWrapper.hpp"

#include <boost/thread.hpp>

#include "LocalMap.hpp"
#include "SpatialQuery.hpp"

SpatialQueryWrapper::
SpatialQueryWrapper() {
  mIsRunning = false;
  mMapSubscription = NULL;
  mNeedsUpdate = false;
  setMapChannel("LOCAL_MAP");
  mMap.reset(new LocalMap());
  mQuery.reset(new SpatialQuery());
  mNewQuery.reset(new SpatialQuery());
}

SpatialQueryWrapper::
~SpatialQueryWrapper() {
  stop();
}

void SpatialQueryWrapper::
setLcm(boost::shared_ptr<lcm::LCM>& iLcm) {
  mLcm = iLcm;
}

void SpatialQueryWrapper::
setMapChannel(const std::string& iChannel) {
  mMapChannel = iChannel;
}

bool SpatialQueryWrapper::
start() {
  if (mIsRunning) {
    return false;
  }
  mIsRunning = true;
  boost::thread thread(boost::ref(*this));
  mMapSubscription =
    mLcm->subscribe(mMapChannel, &SpatialQueryWrapper::onMap, this);
  return true;
}

bool SpatialQueryWrapper::
stop() {
  if (!mIsRunning) {
    return false;
  }
  mDataReady.notify_all();
  mLcm->unsubscribe(mMapSubscription);
  mMapSubscription = NULL;
  mIsRunning = false;
  return true;
}

void SpatialQueryWrapper::
operator()() {
  while (mIsRunning) {
    boost::mutex::scoped_lock mapLock(mMapMutex);
    while (!mNeedsUpdate) {
      mDataReady.wait(mapLock);
    }
    if (mNeedsUpdate) {
      LocalMap::PointCloud::Ptr cloud = mMap->getAsPointCloud();
      mNewQuery->clear();
      mNewQuery->setNormalComputationRadius(3*mMap->getResolution());
      mNewQuery->setMap(mMap);
      mNewQuery->populateStructures();
      {
        boost::mutex::scoped_lock queryLock(mQueryMutex);
        std::swap(mQuery, mNewQuery);
      }
      mNeedsUpdate = false;
    }
  }
}

void SpatialQueryWrapper::
onMap(const lcm::ReceiveBuffer* iBuf,
      const std::string& iChannel,
      const drc::local_map_t* iMessage) {
  if ((iMessage->id == mMap->getId()) &&
      (iMessage->state_id == mMap->getStateId())) {
    return;
  }

  std::vector<char> bytes(iMessage->data.begin(), iMessage->data.end());
  mMap->deserialize(bytes);
  mNeedsUpdate = true;
  mDataReady.notify_one();
}

boost::shared_ptr<SpatialQuery> SpatialQueryWrapper::
query() {
  return mQuery;
}

bool SpatialQueryWrapper::
lock() {
  mQueryMutex.lock();
  return true;
}

bool SpatialQueryWrapper::
unlock() {
  mQueryMutex.unlock();
  return true;
}
