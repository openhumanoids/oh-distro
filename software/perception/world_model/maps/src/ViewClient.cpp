#include "ViewClient.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm_coretypes.h>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/map_catalog_t.hpp>
#include <thread>
#include <drc_utils/Clock.hpp>

#include "PointCloudView.hpp"
#include "OctreeView.hpp"
#include "DepthImageView.hpp"
#include "LcmTranslator.hpp"
#include "ThreadSafeQueue.hpp"
#include "BotWrapper.hpp"
#include "Utils.hpp"

using namespace maps;

struct ViewClient::Worker {

  enum State {
    StateIdle,
    StateRunning,
    StateShutdown
  };

  struct Message {
    typedef std::shared_ptr<Message> Ptr;
    std::vector<uint8_t> mBytes;
  };
  
  ViewClient* mClient;
  BotWrapper::Ptr mBotWrapper;
  State mState;
  std::vector<lcm::Subscription*> mViewSubscriptions;
  lcm::Subscription* mCatalogSubscription;
  ThreadSafeQueue<Message::Ptr> mMessageQueue;
  std::mutex mMutex;
  std::condition_variable mCondition;
  std::thread mThread;

  Worker(ViewClient* iClient) {
    mClient = iClient;
    mCatalogSubscription = NULL;
    mState = StateIdle;
    mThread = std::thread(std::ref(*this));
  }

  ~Worker() {
    std::unique_lock<std::mutex> lock(mMutex);
    if (mBotWrapper != NULL) {
      if (mBotWrapper->getLcm() != NULL) {
        for (int i = 0; i < mViewSubscriptions.size(); ++i) {
          mBotWrapper->getLcm()->unsubscribe(mViewSubscriptions[i]);
        }
        mBotWrapper->getLcm()->unsubscribe(mCatalogSubscription);
      }
    }
    mState = StateShutdown;
    lock.unlock();
    mCondition.notify_one();
    mMessageQueue.unblock();
    mThread.join();
  }

  bool start() {
    std::unique_lock<std::mutex> lock(mMutex);
    if (mState == StateRunning) {
      return false;
    }
    if ((mBotWrapper == NULL) || (mBotWrapper->getLcm() == NULL)) {
      return false;
    }
    mState = StateRunning;
    lock.unlock();
    mCondition.notify_one();
    for (int i = 0; i < mClient->mViewChannels.size(); ++i) {
      lcm::Subscription* sub = mBotWrapper->getLcm()->
        subscribe(mClient->mViewChannels[i], &Worker::onView, this);
      mViewSubscriptions.push_back(sub);
    }
    mCatalogSubscription = mBotWrapper->getLcm()->
      subscribe(mClient->mCatalogChannel, &Worker::onView, this);
    return true;
  }

  bool stop() {
    std::unique_lock<std::mutex> lock(mMutex);
    if (mState != StateRunning) {
      return false;
    }
    if (mBotWrapper != NULL) {
      if (mBotWrapper->getLcm() != NULL) {
        for (int i = 0; i < mViewSubscriptions.size(); ++i) {
          mBotWrapper->getLcm()->unsubscribe(mViewSubscriptions[i]);
        }
        mBotWrapper->getLcm()->unsubscribe(mCatalogSubscription);
      }
    }
    mState = StateIdle;
    lock.unlock();
    mCondition.notify_one();
    mMessageQueue.clear();
    return true;
  }

  void onView(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel) {
    Message::Ptr msg(new Message);
    const uint8_t* buf = static_cast<const uint8_t*>(iBuf->data);
    msg->mBytes = std::vector<uint8_t>(buf, buf + iBuf->data_size);
    mMessageQueue.push(msg);
  }

  void operator()() {
    while (mState != StateShutdown) {

      {
        std::unique_lock<std::mutex> lock(mMutex);
        while (mState == StateIdle) {
          mCondition.wait(lock);
        }
      }

      if (mState != StateRunning) {
        continue;
      }

      // wait for new message
      Message::Ptr msg;
      if (!mMessageQueue.waitForData(msg)) {
        continue;
      }

      // get hash of raw buffer
      const void* buf = static_cast<const void*>(&(msg->mBytes[0]));
      int64_t hash;
      int len = __int64_t_decode_array(buf, 0, 8, &hash, 1);
      if (len < 0) continue;
      const int maxBufferSize = 1000000;

      // handle catalog
      if (hash == drc::map_catalog_t::getHash()) {
        drc::map_catalog_t catalog;
        catalog.decode(buf, 0, maxBufferSize);
        bool changed = false;
        std::set<int64_t> catalogIds;
        for (int i = 0; i < catalog.views.size(); ++i) {
          ViewBase::Spec spec;
          LcmTranslator::fromLcm(catalog.views[i], spec);
          int64_t id = spec.mViewId;
          catalogIds.insert(id);
          SpecCollection::const_iterator item = mClient->mCatalog.find(id);
          if ((item == mClient->mCatalog.end()) || (spec != item->second)) {
            changed = true;
          }
          mClient->mCatalog[id] = spec;
        }
        for (ViewCollection::const_iterator iter = mClient->mViews.begin();
             iter != mClient->mViews.end(); ) {
          if (catalogIds.find(iter->second->getId()) == catalogIds.end()) {
            mClient->mViews.erase(iter++);
            changed = true;
          }
          else {
            ++iter;
          }
        }
        for (SpecCollection::const_iterator iter = mClient->mCatalog.begin();
             iter != mClient->mCatalog.end(); ) {
          if (catalogIds.find(iter->second.mViewId) == catalogIds.end()) {
            mClient->mCatalog.erase(iter++);
            changed = true;
          }
          else {
            ++iter;
          }
        }
        mClient->notifyCatalogListeners(changed);
        continue;
      }

      ViewPtr view;

      // handle octree
      if (hash == drc::map_octree_t::getHash()) {
        drc::map_octree_t octree;
        octree.decode(buf, 0, maxBufferSize);
        OctreeView* octreeView = new OctreeView();
        LcmTranslator::fromLcm(octree, *octreeView);
        view.reset(octreeView);
        view->setUpdateTime(octree.utime);
      }

      // handle cloud
      else if (hash == drc::map_cloud_t::getHash()) {
        drc::map_cloud_t cloud;
        cloud.decode(buf, 0, maxBufferSize);
        PointCloudView* cloudView = new PointCloudView();
        LcmTranslator::fromLcm(cloud, *cloudView);
        view.reset(cloudView);
        view->setUpdateTime(cloud.utime);
      }

      // handle depth image
      else if (hash == drc::map_image_t::getHash()) {
        drc::map_image_t image;
        image.decode(buf, 0, maxBufferSize);
        DepthImageView* depthView = new DepthImageView();
        LcmTranslator::fromLcm(image, *depthView);
        view.reset(depthView);
        view->setUpdateTime(image.utime);
      }

      if (view == NULL) return;

      // find view or insert new one for this id
      SpecCollection::const_iterator item =
        mClient->mCatalog.find(view->getId());
      if (item == mClient->mCatalog.end()) {
        ViewBase::Spec spec;
        spec.mViewId = view->getId();
        mClient->mCatalog[view->getId()] = spec;
        mClient->notifyCatalogListeners(true);
      }
      mClient->mViews[view->getId()] = view;

      // notify subscribers
      mClient->notifyDataListeners(view->getId());
    }
  }
};

ViewClient::
ViewClient() {
  setRequestChannel("MAP_REQUEST");
  setCatalogChannel("MAP_CATALOG");
  addViewChannel("MAP_OCTREE");
  addViewChannel("MAP_CLOUD");
  addViewChannel("MAP_DEPTH");
  mWorker.reset(new Worker(this));
}

ViewClient::
~ViewClient() {
  stop();
}

void ViewClient::
setBotWrapper(const std::shared_ptr<BotWrapper>& iWrapper) {
  mBotWrapper = iWrapper;
  mWorker->mBotWrapper = iWrapper;
}

void ViewClient::
setRequestChannel(const std::string& iChannel) {
  mRequestChannel = iChannel;
}

void ViewClient::
setCatalogChannel(const std::string& iChannel) {
  mCatalogChannel = iChannel;
}

void ViewClient::
addViewChannel(const std::string& iChannel) {
  mViewChannels.push_back(iChannel);
}

void ViewClient::
removeViewChannel(const std::string& iChannel) {
  // TODO
}

void ViewClient::removeAllViewChannels() {
  mViewChannels.clear();
}

int64_t ViewClient::request(const ViewBase::Spec& iSpec) {
  drc::map_request_t message;
  LcmTranslator::toLcm(iSpec, message);
  message.utime = drc::Clock::instance()->getCurrentTime();
  if (message.view_id < 0) {
    message.view_id = (Utils::rand64() >> 1);
  }
  mBotWrapper->getLcm()->publish(mRequestChannel, &message);
  return message.view_id;
}

ViewClient::ViewPtr ViewClient::
getView(const int64_t iId) const {
  ViewCollection::const_iterator item = mViews.find(iId);
  if (item != mViews.end()) {
    return item->second;
  }
  return ViewPtr();
}

std::vector<ViewClient::ViewPtr> ViewClient::
getAllViews() const {
  std::vector<ViewPtr> views;
  views.reserve(mViews.size());
  ViewCollection::const_iterator iter;
  for (iter = mViews.begin(); iter != mViews.end(); ++iter) {
    views.push_back(iter->second);
  }
  return views;
}

bool ViewClient::
getSpec(const int64_t iId, ViewBase::Spec& oSpec) const {
  SpecCollection::const_iterator item = mCatalog.find(iId);
  if (item == mCatalog.end()) {
    return false;
  }
  oSpec = item->second;
  return true;
}

std::vector<ViewBase::Spec> ViewClient::
getAllSpecs() const {
  std::vector<ViewBase::Spec> specs;
  specs.reserve(mCatalog.size());
  SpecCollection::const_iterator iter;
  for (iter = mCatalog.begin(); iter != mCatalog.end(); ++iter) {
    specs.push_back(iter->second);
  }
  return specs;
}

void ViewClient::
clearAll() {
  mViews.clear();
  mCatalog.clear();
  notifyCatalogListeners(true);
}


bool ViewClient::
addListener(const Listener* iListener) {
  return (mListeners.insert((Listener*)iListener).second);
}

bool ViewClient::
removeListener(const Listener* iListener) {
  return (mListeners.erase((Listener*)iListener) > 0);
}

bool ViewClient::
removeAllListeners() {
  mListeners.clear();
  return true;
}

bool ViewClient::
start() {
  return mWorker->start();
}

bool ViewClient::
stop() {
  return mWorker->stop();
}

void ViewClient::
notifyCatalogListeners(const bool iChanged) {
  for (std::set<Listener*>::iterator iter = mListeners.begin();
       iter != mListeners.end(); ++iter) {
    if (*iter != NULL) {
      (*iter)->notifyCatalog(iChanged);
    }
  }
}

void ViewClient::
notifyDataListeners(const int64_t iId) {
  for (std::set<Listener*>::iterator iter = mListeners.begin();
       iter != mListeners.end(); ++iter) {
    if (*iter != NULL) {
      (*iter)->notifyData(iId);
    }
  }
}
