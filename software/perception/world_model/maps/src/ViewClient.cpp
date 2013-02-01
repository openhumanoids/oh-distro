#include "ViewClient.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "LcmTranslator.hpp"
#include "ThreadSafeQueue.hpp"

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
      TypeCloud
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
  ThreadSafeQueue<Message> mMessageQueue;
  boost::mutex mMutex;
  boost::condition_variable mCondition;
  boost::thread mThread;

  Worker(ViewClient* iClient) {
    mClient = iClient;
    mOctreeSubscription = NULL;
    mCloudSubscription = NULL;
    mState = StateIdle;
    mThread = boost::thread(boost::ref(*this));
  }

  ~Worker() {
    boost::mutex::scoped_lock lock(mMutex);
    mLcm->unsubscribe(mOctreeSubscription);
    mLcm->unsubscribe(mCloudSubscription);
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
    return true;
  }

  bool stop() {
    boost::mutex::scoped_lock lock(mMutex);
    if (mState != StateRunning) {
      return false;
    }
    mLcm->unsubscribe(mOctreeSubscription);
    mLcm->unsubscribe(mCloudSubscription);
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

      // find view or insert new one for this id
      MapViewCollection::const_iterator item = mClient->mViews.find(msg.mId);
      if (item == mClient->mViews.end()) {
        MapView::Spec spec;
        spec.mViewId = msg.mId;
        mClient->mViews[msg.mId].reset(new MapView(spec));
      }
      MapViewPtr view = mClient->mViews[msg.mId];

      // handle octree
      if (msg.mType == Message::TypeOctree) {
        boost::shared_ptr<drc::map_octree_t> payload =
          boost::static_pointer_cast<drc::map_octree_t>(msg.mPayload);
        maps::Octree octree = LcmTranslator::fromLcm(*payload);
        view->set(octree);
      }

      // handle cloud
      else if (msg.mType == Message::TypeCloud) {
        boost::shared_ptr<drc::map_cloud_t> payload =
          boost::static_pointer_cast<drc::map_cloud_t>(msg.mPayload);
        maps::PointCloud::Ptr cloud = LcmTranslator::fromLcm(*payload);
        view->set(*cloud);
      }

      // notify all listeners
      for (std::set<Listener*>::iterator iter = mClient->mListeners.begin();
           iter != mClient->mListeners.end(); ++iter) {
        if (*iter != NULL) {
          (*iter)->notify(msg.mId);
        }
      }
    }
  }
};

ViewClient::
ViewClient() {
  setOctreeChannel("MAP_OCTREE");
  setCloudChannel("MAP_CLOUD");
  mWorker.reset(new Worker(this));
}

ViewClient::
~ViewClient() {
  stop();
}

void ViewClient::
setLcm(const LcmPtr& iLcm) {
  mWorker->mLcm = iLcm;
}

void ViewClient::
setOctreeChannel(const std::string& iChannel) {
  mOctreeChannel = iChannel;
}

void ViewClient::
setCloudChannel(const std::string& iChannel) {
  mCloudChannel = iChannel;
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
