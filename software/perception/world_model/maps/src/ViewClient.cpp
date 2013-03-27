#include "ViewClient.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/map_catalog_t.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
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
    enum Type {
      TypeOctree,
      TypeCloud,
      TypeDepth,
      TypeCatalog
    };
    Type mType;
    int64_t mId;
    boost::shared_ptr<void> mPayload;
  };
  
  ViewClient* mClient;
  BotWrapper::Ptr mBotWrapper;
  State mState;
  lcm::Subscription* mOctreeSubscription;
  lcm::Subscription* mCloudSubscription;
  lcm::Subscription* mDepthSubscription;
  lcm::Subscription* mCatalogSubscription;
  ThreadSafeQueue<Message> mMessageQueue;
  boost::mutex mMutex;
  boost::condition_variable mCondition;
  boost::thread mThread;

  Worker(ViewClient* iClient) {
    mClient = iClient;
    mOctreeSubscription = NULL;
    mCloudSubscription = NULL;
    mDepthSubscription = NULL;
    mCatalogSubscription = NULL;
    mState = StateIdle;
    mThread = boost::thread(boost::ref(*this));
  }

  ~Worker() {
    boost::mutex::scoped_lock lock(mMutex);
    if (mBotWrapper != NULL) {
      if (mBotWrapper->getLcm() != NULL) {
        mBotWrapper->getLcm()->unsubscribe(mOctreeSubscription);
        mBotWrapper->getLcm()->unsubscribe(mCloudSubscription);
        mBotWrapper->getLcm()->unsubscribe(mDepthSubscription);
        mBotWrapper->getLcm()->unsubscribe(mCatalogSubscription);
      }
    }
    mState = StateShutdown;
    lock.unlock();
    mCondition.notify_one();
    mMessageQueue.unblock();
    try { mThread.join(); }
    catch (const boost::thread_interrupted&) {}
  }

  bool start() {
    boost::mutex::scoped_lock lock(mMutex);
    if (mState == StateRunning) {
      return false;
    }
    if ((mBotWrapper == NULL) || (mBotWrapper->getLcm() == NULL)) {
      return false;
    }
    mState = StateRunning;
    mCondition.notify_one();
    lock.unlock();
    mOctreeSubscription = mBotWrapper->getLcm()->
      subscribe(mClient->mOctreeChannel, &Worker::onOctree, this);
    mCloudSubscription = mBotWrapper->getLcm()->
      subscribe(mClient->mCloudChannel, &Worker::onCloud, this);
    mDepthSubscription = mBotWrapper->getLcm()->
      subscribe(mClient->mDepthChannel, &Worker::onDepth, this);
    mCatalogSubscription = mBotWrapper->getLcm()->
      subscribe(mClient->mCatalogChannel, &Worker::onCatalog, this);
    return true;
  }

  bool stop() {
    boost::mutex::scoped_lock lock(mMutex);
    if (mState != StateRunning) {
      return false;
    }
    if (mBotWrapper != NULL) {
      if (mBotWrapper->getLcm() != NULL) {
        mBotWrapper->getLcm()->unsubscribe(mOctreeSubscription);
        mBotWrapper->getLcm()->unsubscribe(mCloudSubscription);
        mBotWrapper->getLcm()->unsubscribe(mDepthSubscription);
        mBotWrapper->getLcm()->unsubscribe(mCatalogSubscription);
      }
    }
    mState = StateIdle;
    lock.unlock();
    mCondition.notify_one();
    mMessageQueue.clear();
    return true;
  }

  void onOctree(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel,
                const drc::map_octree_t* iMessage) {
    Message msg;
    msg.mType = Message::TypeOctree;
    msg.mId = iMessage->view_id;
    boost::shared_ptr<drc::map_octree_t>
      payload(new drc::map_octree_t(*iMessage));
    msg.mPayload = boost::static_pointer_cast<void>(payload);
    mMessageQueue.push(msg);
  }

  void onCloud(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const drc::map_cloud_t* iMessage) {
    Message msg;
    msg.mType = Message::TypeCloud;
    msg.mId = iMessage->view_id;
    boost::shared_ptr<drc::map_cloud_t>
      payload(new drc::map_cloud_t(*iMessage));
    msg.mPayload = boost::static_pointer_cast<void>(payload);
    mMessageQueue.push(msg);
  }

  void onDepth(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const drc::map_image_t* iMessage) {
    Message msg;
    msg.mType = Message::TypeDepth;
    msg.mId = iMessage->view_id;
    boost::shared_ptr<drc::map_image_t>
      payload(new drc::map_image_t(*iMessage));
    msg.mPayload = boost::static_pointer_cast<void>(payload);
    mMessageQueue.push(msg);
  }

  void onCatalog(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const drc::map_catalog_t* iMessage) {
    Message msg;
    msg.mType = Message::TypeCatalog;
    msg.mId = 0;
    boost::shared_ptr<drc::map_catalog_t>
      payload(new drc::map_catalog_t(*iMessage));
    msg.mPayload = boost::static_pointer_cast<void>(payload);
    mMessageQueue.push(msg);
  }

  void operator()() {
    while (mState != StateShutdown) {

      {
        boost::mutex::scoped_lock lock(mMutex);
        while (mState == StateIdle) {
          mCondition.wait(lock);
        }
      }

      if (mState != StateRunning) {
        continue;
      }

      // wait for new message
      Message msg;
      if (!mMessageQueue.waitForData(msg)) {
        continue;
      }

      // handle catalog
      if (msg.mType == Message::TypeCatalog) {
        boost::shared_ptr<drc::map_catalog_t> catalog =
          boost::static_pointer_cast<drc::map_catalog_t>(msg.mPayload);
        bool changed = false;
        std::set<int64_t> catalogIds;
        for (int i = 0; i < catalog->views.size(); ++i) {
          ViewBase::Spec spec;
          LcmTranslator::fromLcm(catalog->views[i], spec);
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

      // find view or insert new one for this id
      SpecCollection::const_iterator item = mClient->mCatalog.find(msg.mId);
      if (item == mClient->mCatalog.end()) {
        ViewBase::Spec spec;
        spec.mViewId = msg.mId;
        mClient->mCatalog[msg.mId] = spec;
        mClient->notifyCatalogListeners(true);
      }
      ViewPtr view;

      // handle octree
      if (msg.mType == Message::TypeOctree) {
        boost::shared_ptr<drc::map_octree_t> payload =
          boost::static_pointer_cast<drc::map_octree_t>(msg.mPayload);
        OctreeView* octreeView = new OctreeView();
        LcmTranslator::fromLcm(*payload, *octreeView);
        view.reset(octreeView);
        view->setUpdateTime(payload->utime);
      }

      // handle cloud
      else if (msg.mType == Message::TypeCloud) {
        boost::shared_ptr<drc::map_cloud_t> payload =
          boost::static_pointer_cast<drc::map_cloud_t>(msg.mPayload);
        PointCloudView* cloudView = new PointCloudView();
        LcmTranslator::fromLcm(*payload, *cloudView);
        view.reset(cloudView);
        view->setUpdateTime(payload->utime);
      }

      // handle depth image
      else if (msg.mType == Message::TypeDepth) { 
        boost::shared_ptr<drc::map_image_t> payload =
          boost::static_pointer_cast<drc::map_image_t>(msg.mPayload);
        DepthImageView* depthView = new DepthImageView();
        LcmTranslator::fromLcm(*payload, *depthView);
        view.reset(depthView);
        view->setUpdateTime(payload->utime);
      }

      if (view != NULL) mClient->mViews[msg.mId] = view;

      // notify subscribers
      mClient->notifyDataListeners(msg.mId);
    }
  }
};

ViewClient::
ViewClient() {
  setRequestChannel("MAP_REQUEST");
  setOctreeChannel("MAP_OCTREE");
  setCloudChannel("MAP_CLOUD");
  setDepthChannel("MAP_DEPTH");
  setCatalogChannel("MAP_CATALOG");
  mWorker.reset(new Worker(this));
}

ViewClient::
~ViewClient() {
  stop();
}

void ViewClient::
setBotWrapper(const boost::shared_ptr<BotWrapper>& iWrapper) {
  mBotWrapper = iWrapper;
  mWorker->mBotWrapper = iWrapper;
}

void ViewClient::
setRequestChannel(const std::string& iChannel) {
  mRequestChannel = iChannel;
}

void ViewClient::
setOctreeChannel(const std::string& iChannel) {
  mOctreeChannel = iChannel;
}

void ViewClient::
setCloudChannel(const std::string& iChannel) {
  mCloudChannel = iChannel;
}

void ViewClient::
setDepthChannel(const std::string& iChannel) {
  mDepthChannel = iChannel;
}

void ViewClient::
setCatalogChannel(const std::string& iChannel) {
  mCatalogChannel = iChannel;
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
