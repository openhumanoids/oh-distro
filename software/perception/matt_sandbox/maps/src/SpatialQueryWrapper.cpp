#include "SpatialQueryWrapper.hpp"

#include <boost/thread.hpp>

#include "MapChunk.hpp"
#include "SpatialQuery.hpp"

SpatialQueryWrapper::
SpatialQueryWrapper() {
  mIsRunning = false;
  mMapSubscription = NULL;
  mNeedsUpdate = false;
  setMapChannel("LOCAL_MAP");
  mMap.reset(new MapChunk());
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
      MapChunk::PointCloud::Ptr cloud = mMap->getAsPointCloud();
      mNewQuery->clear();
      mNewQuery->setNormalComputationRadius(3*mMap->getResolution());
      mNewQuery->add(cloud);
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
      const bot_core::raw_t* iMessage) {
  std::vector<char> bytes(iMessage->data.begin(), iMessage->data.end());
  mMap->deserialize(bytes);
  mNeedsUpdate = true;
  mDataReady.notify_one();
}

bool SpatialQueryWrapper::
getClosest(const Eigen::Vector3d& iPoint,
           Eigen::Vector3d& oPoint) {
  return mQuery->getClosest(iPoint, oPoint);
}

bool SpatialQueryWrapper::
getClosest(const Eigen::Vector3d& iPoint,
           Eigen::Vector3d& oPoint, Eigen::Vector3d& oNormal) {
  return mQuery->getClosest(iPoint, oPoint, oNormal);
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
