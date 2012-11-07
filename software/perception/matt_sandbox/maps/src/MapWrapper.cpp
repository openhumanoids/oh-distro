#include "MapWrapper.hpp"

#include <boost/thread.hpp>

#include "LocalMap.hpp"

MapWrapper::
MapWrapper() {
  mIsRunning = false;
  mMapSubscription = NULL;
  mNeedsUpdate = false;
  setMapChannel("LOCAL_MAP");
  mMap.reset(new LocalMap());
  mNewMap.reset(new LocalMap());
}

MapWrapper::
~MapWrapper() {
  stop();
}

void MapWrapper::
setLcm(boost::shared_ptr<lcm::LCM>& iLcm) {
  mLcm = iLcm;
}

void MapWrapper::
setMapChannel(const std::string& iChannel) {
  mMapChannel = iChannel;
}

bool MapWrapper::
start() {
  if (mIsRunning) {
    return false;
  }
  mIsRunning = true;
  boost::thread thread(boost::ref(*this));
  mMapSubscription = mLcm->subscribe(mMapChannel, &MapWrapper::onMap, this);
  return true;
}

bool MapWrapper::
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

void MapWrapper::
operator()() {
  while (mIsRunning) {
    boost::mutex::scoped_lock lock(mDataReadyMutex);
    while (!mNeedsUpdate) {
      mDataReady.wait(lock);
    }
    if (mNeedsUpdate) {
      boost::mutex::scoped_lock mapLock(mMapMutex);
      boost::mutex::scoped_lock newMapLock(mNewMapMutex);
      std::swap(mMap, mNewMap);
      mNeedsUpdate = false;
    }
  }
}

void MapWrapper::
onMap(const lcm::ReceiveBuffer* iBuf,
      const std::string& iChannel,
      const drc::local_map_t* iMessage) {
  if ((iMessage->id == mNewMap->getId()) &&
      (iMessage->state_id == mNewMap->getStateId())) {
    return;
  }

  std::vector<char> bytes(iMessage->data.begin(), iMessage->data.end());
  boost::mutex::scoped_lock lock(mNewMapMutex);
  mNewMap->deserialize(bytes);
  mNeedsUpdate = true;
  mDataReady.notify_one();
}

MapWrapper::LocalMapConstPtr MapWrapper::
getMap() const {
  return mMap;
}

bool MapWrapper::
lock() {
  mMapMutex.lock();
  return true;
}

bool MapWrapper::
unlock() {
  mMapMutex.unlock();
  return true;
}
