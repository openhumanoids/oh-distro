#include "ViewClient.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_catalog_t.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <drc_utils/Clock.hpp>

#include "LcmTranslator.hpp"
#include "ThreadSafeQueue.hpp"
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
      TypeCatalog
    };
    Type mType;
    int64_t mId;
    boost::shared_ptr<void> mPayload;
  };

  ViewClient* mClient;
  LcmPtr mLcm;
  State mState;
  lcm::Subscription* mOctreeSubscription;
  lcm::Subscription* mCloudSubscription;
  lcm::Subscription* mCatalogSubscription;
  ThreadSafeQueue<Message> mMessageQueue;
  boost::mutex mMutex;
  boost::condition_variable mCondition;
  boost::thread mThread;

  Worker(ViewClient* iClient) {
    mClient = iClient;
    mOctreeSubscription = NULL;
    mCloudSubscription = NULL;
    mCatalogSubscription = NULL;
    mState = StateIdle;
    mThread = boost::thread(boost::ref(*this));
  }

  ~Worker() {
    boost::mutex::scoped_lock lock(mMutex);
    mLcm->unsubscribe(mOctreeSubscription);
    mLcm->unsubscribe(mCloudSubscription);
    mLcm->unsubscribe(mCatalogSubscription);
    mState = StateShutdown;
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
    mState = StateRunning;
    mCondition.notify_one();
    mOctreeSubscription =
      mLcm->subscribe(mClient->mOctreeChannel, &Worker::onOctree, this);
    mCloudSubscription =
      mLcm->subscribe(mClient->mCloudChannel, &Worker::onCloud, this);
    mCatalogSubscription =
      mLcm->subscribe(mClient->mCatalogChannel, &Worker::onCatalog, this);
    return true;
  }

  bool stop() {
    boost::mutex::scoped_lock lock(mMutex);
    if (mState != StateRunning) {
      return false;
    }
    mLcm->unsubscribe(mOctreeSubscription);
    mLcm->unsubscribe(mCloudSubscription);
    mLcm->unsubscribe(mCatalogSubscription);
    mMessageQueue.unblock();
    mState = StateIdle;
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
          MapView::Spec spec = LcmTranslator::fromLcm(catalog->views[i]);
          int64_t id = spec.mViewId;
          catalogIds.insert(id);
          MapViewCollection::const_iterator item = mClient->mViews.find(id);
          if (item == mClient->mViews.end()) {
            mClient->mViews[id].reset(new MapView(spec));
            changed = true;
          }
          else {
            if (spec != item->second->getSpec()) {
              mClient->mViews[id] = item->second->clone(spec);
              changed = true;
            }
          }
        }
        for (MapViewCollection::const_iterator iter = mClient->mViews.begin();
             iter != mClient->mViews.end(); ) {
          if (catalogIds.find(iter->second->getSpec().mViewId) ==
              catalogIds.end()) {
            mClient->mViews.erase(iter++);
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
      MapViewCollection::const_iterator item = mClient->mViews.find(msg.mId);
      if (item == mClient->mViews.end()) {
        MapView::Spec spec;
        spec.mViewId = msg.mId;
        mClient->mViews[msg.mId].reset(new MapView(spec));
        mClient->notifyCatalogListeners(true);
      }
      MapViewPtr view = mClient->mViews[msg.mId];

      // handle octree
      if (msg.mType == Message::TypeOctree) {
        boost::shared_ptr<drc::map_octree_t> payload =
          boost::static_pointer_cast<drc::map_octree_t>(msg.mPayload);
        maps::Octree octree = LcmTranslator::fromLcm(*payload);
        view->set(octree);
        view->setUpdateTime(payload->utime);
      }

      // handle cloud
      else if (msg.mType == Message::TypeCloud) {
        boost::shared_ptr<drc::map_cloud_t> payload =
          boost::static_pointer_cast<drc::map_cloud_t>(msg.mPayload);
        maps::PointCloud::Ptr cloud = LcmTranslator::fromLcm(*payload);
        view->set(*cloud);
        view->setUpdateTime(payload->utime);
      }

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
  setCatalogChannel("MAP_CATALOG");
  mWorker.reset(new Worker(this));
}

ViewClient::
~ViewClient() {
  stop();
}

void ViewClient::
setLcm(const LcmPtr& iLcm) {
  mLcm = iLcm;
  mWorker->mLcm = iLcm;
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
setCatalogChannel(const std::string& iChannel) {
  mCatalogChannel = iChannel;
}

int64_t ViewClient::request(const MapView::Spec& iSpec) {
  drc::map_request_t message = LcmTranslator::toLcm(iSpec);
  message.utime = drc::Clock::instance()->getCurrentTime();
  if (message.view_id < 0) {
    message.view_id = (Utils::rand64() >> 1);
  }
  mLcm->publish(mRequestChannel, &message);
  return message.view_id;
}

ViewClient::MapViewPtr ViewClient::
getView(const int64_t iId) const {
  MapViewCollection::const_iterator item = mViews.find(iId);
  if (item != mViews.end()) {
    return item->second;
  }
  return MapViewPtr();
}

std::vector<ViewClient::MapViewPtr> ViewClient::
getAllViews() const {
  std::vector<MapViewPtr> views;
  views.reserve(mViews.size());
  MapViewCollection::const_iterator iter;
  for (iter = mViews.begin(); iter != mViews.end(); ++iter) {
    views.push_back(iter->second);
  }
  return views;
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
