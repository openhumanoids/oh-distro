#include "ViewClient.hpp"

#include <iostream>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm_coretypes.h>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>
#include <drc_utils/BotWrapper.hpp>

#include "Utils.hpp"
#include "DepthImageView.hpp"
#include "LcmTranslator.hpp"
#include "ThreadSafeQueue.hpp"
#include "ObjectPool.hpp"

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
  drc::BotWrapper::Ptr mBotWrapper;
  bool mRunning;
  std::vector<lcm::Subscription*> mViewSubscriptions;
  lcm::Subscription* mCatalogSubscription;
  ThreadSafeQueue<Message::Ptr> mMessageQueue;
  std::mutex mMutex;
  std::thread mThread;

  ObjectPool<DepthImageView,20> mDepthImageViewPool;

  Worker(ViewClient* iClient) {
    mClient = iClient;
    mBotWrapper = iClient->mBotWrapper;
    mCatalogSubscription = NULL;
    mRunning = false;
  }

  ~Worker() {
    stop();
  }

  bool start() {
    if (mRunning) {
      return false;
    }
    if ((mBotWrapper == NULL) || (mBotWrapper->getLcm() == NULL)) {
      return false;
    }

    mRunning = true;
    mThread = std::thread(std::ref(*this));
    for (int i = 0; i < (int)mClient->mViewChannels.size(); ++i) {
      lcm::Subscription* sub = mBotWrapper->getLcm()->
        subscribe(mClient->mViewChannels[i], &Worker::onView, this);
      mViewSubscriptions.push_back(sub);
    }
    mCatalogSubscription = mBotWrapper->getLcm()->
      subscribe(mClient->mCatalogChannel, &Worker::onView, this);
    return true;
  }

  bool stop() {
    mRunning = false;
    mMessageQueue.clear();
    if (mThread.joinable()) mThread.join();

    if (mBotWrapper != NULL) {
      if (mBotWrapper->getLcm() != NULL) {
        for (int i = 0; i < (int)mViewSubscriptions.size(); ++i) {
          mBotWrapper->getLcm()->unsubscribe(mViewSubscriptions[i]);
        }
        mBotWrapper->getLcm()->unsubscribe(mCatalogSubscription);
      }
    }
    return true;
  }

  void onView(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel) {
    Message::Ptr msg(new Message);
    const uint8_t* buf = static_cast<const uint8_t*>(iBuf->data);
    msg->mBytes = std::vector<uint8_t>(buf, buf + iBuf->data_size);
    mMessageQueue.push(msg);
  }

  void operator()() {
    while (mRunning) {

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

      ViewPtr view;

      // handle depth image
      if (hash == drc::map_image_t::getHash()) {
        drc::map_image_t image;
        image.decode(buf, 0, msg->mBytes.size());
        auto depthView = mDepthImageViewPool.get();
        if (depthView == NULL) {
          std::cout << "Warning: no objects in depthimage pool" << std::endl;
          continue;          
        }
        LcmTranslator::fromLcm(image, *depthView);
        view = depthView;
        view->setUpdateTime(image.utime);
      }

      if (view == NULL) continue;

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
  addViewChannel("MAP_DEPTH");
}

ViewClient::
~ViewClient() {
  stop();
}

void ViewClient::
setBotWrapper(const std::shared_ptr<drc::BotWrapper>& iWrapper) {
  mBotWrapper = iWrapper;
  if (mWorker != NULL) mWorker->mBotWrapper = iWrapper;
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
  message.utime = mBotWrapper->getCurrentTime();
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

bool ViewClient::
removeView(const int64_t iId) {
  std::unique_lock<std::mutex> lock(mWorker->mMutex);
  if (mViews.find(iId) == mViews.end()) return false;
  mViews.erase(iId);
  if (mCatalog.find(iId) != mCatalog.end()) mCatalog.erase(iId);
  return true;
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
  if (mWorker != NULL) return false;
  mWorker.reset(new Worker(this));
  return mWorker->start();
}

bool ViewClient::
stop() {
  if (mWorker != NULL) {
    mWorker.reset();
    return true;
  }
  return false;
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
