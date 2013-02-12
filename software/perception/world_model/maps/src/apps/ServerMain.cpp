#include <unordered_map>

#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>
#include <lcmtypes/drc/map_command_t.hpp>
#include <lcmtypes/drc/map_params_t.hpp>
#include <lcmtypes/drc/map_catalog_t.hpp>

#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/SensorDataReceiver.hpp>
#include <maps/DataBlob.hpp>
#include <maps/LcmTranslator.hpp>


#include <drc_utils/Clock.hpp>
#include <pcl/common/transforms.h>
#include <ConciseArgs>
#include <octomap/octomap.h>

// TODO: TEMP
#include <boost/progress.hpp>


using namespace std;
using namespace maps;

struct Worker {
  typedef boost::shared_ptr<Worker> Ptr;

  bool mActive;
  drc::map_request_t mRequest;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::thread mThread;

  ~Worker() {
    stop();
    try { mThread.join(); }
    catch (const boost::thread_interrupted&) {}
  }

  bool start() {
    if (mActive) {
      return false;
    }
    mActive = true;
    mThread = boost::thread(boost::ref(*this));
    return true;
  }

  bool stop() {
    if (!mActive) {
      return false;
    }
    mActive = false;
    return true;
  }

  void operator()() {
    mActive = true;
    while (mActive) {
      // get map
      LocalMap::Ptr localMap;
      if (mRequest.map_id == 0) {
        vector<int64_t> ids = mManager->getAllMapIds(true);
        if (ids.size() > 0) {
          localMap = mManager->getMap(ids.back());
        }
      }
      else {
        localMap = mManager->getMap(mRequest.map_id);
      }
      if (localMap != NULL) {

        // get bounds
        LocalMap::SpaceTimeBounds bounds;
        bounds.mMinTime = mRequest.time_min;
        bounds.mMaxTime = mRequest.time_max;
        bounds.mPlanes.resize(mRequest.num_clip_planes);
        for (int i = 0; i < bounds.mPlanes.size(); ++i) {
          bounds.mPlanes[i] =
            Eigen::Vector4f(mRequest.clip_planes[i][0],
                            mRequest.clip_planes[i][1],
                            mRequest.clip_planes[i][2],
                            mRequest.clip_planes[i][3]);
        }

        // get and publish octree
        if (mRequest.type == drc::map_request_t::OCTREE) {
          maps::Octree octree;
          octree = localMap->getAsOctree(mRequest.resolution, false,
                                         Eigen::Vector3f(0,0,0), bounds);
          std::cout << "Publishing octomap..." << std::endl;
          drc::map_octree_t octMsg = LcmTranslator::toLcm(octree);
          octMsg.utime = drc::Clock::instance()->getCurrentTime();
          octMsg.map_id = localMap->getId();
          octMsg.view_id = mRequest.view_id;
          mLcm->publish("MAP_OCTREE", &octMsg);
          std::cout << "Sent octree at " << octMsg.num_bytes << " bytes" << std::endl;
        }

        else if (mRequest.type == drc::map_request_t::CLOUD) {

          // get point cloud
          maps::PointCloud::Ptr cloud =
            localMap->getAsPointCloud(mRequest.resolution, bounds);

          drc::map_cloud_t msgCloud = LcmTranslator::toLcm(*cloud);
          msgCloud.utime = drc::Clock::instance()->getCurrentTime();
          msgCloud.map_id = localMap->getId();
          msgCloud.view_id = mRequest.view_id;
          msgCloud.blob.utime = msgCloud.utime;
          mLcm->publish("MAP_CLOUD", &msgCloud);
        }

        // one-shot request has 0 frequency
        if (fabs(mRequest.frequency) < 1e-6) {
          mActive = false;
          break;
        }
      }

      // wait for timer expiry
      boost::asio::io_service service;
      boost::asio::deadline_timer timer(service);
      timer.expires_from_now(boost::posix_time::
                             milliseconds(1000/mRequest.frequency));
      timer.wait();
      std::cout << "Timer expired for view " << mRequest.view_id << std::endl;
    }
  }
};

typedef std::unordered_map<int64_t,Worker::Ptr> WorkerMap;

class State {
public:
  boost::shared_ptr<SensorDataReceiver> mSensorDataReceiver;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  WorkerMap mWorkers;

  lcm::Subscription* mRequestSubscription;
  lcm::Subscription* mMapParamsSubscription;
  lcm::Subscription* mMapCommandSubscription;

  float mCatalogPublishPeriod;

  State() {
    mSensorDataReceiver.reset(new SensorDataReceiver());
    mManager.reset(new MapManager());
    mLcm.reset(new lcm::LCM());
    drc::Clock::instance()->setLcm(mLcm);
    mSensorDataReceiver->setLcm(mLcm);
    mRequestSubscription = NULL;
    mCatalogPublishPeriod = 10;
  }

  ~State() {
    mLcm->unsubscribe(mRequestSubscription);
    mLcm->unsubscribe(mMapParamsSubscription);
    mLcm->unsubscribe(mMapCommandSubscription);
  }

  void onRequest(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const drc::map_request_t* iMessage) {
    addWorker(*iMessage);
  }

  void onMapCommand(const lcm::ReceiveBuffer* iBuf,
                    const std::string& iChannel,
                    const drc::map_command_t* iMessage) {
    switch (iMessage->command) {
    case drc::map_command_t::CLEAR:
      mManager->clearMap(iMessage->map_id);
      break;
    case drc::map_command_t::STOP:
      mManager->stopUpdatingMap(iMessage->map_id);
      break;
    case drc::map_command_t::START:
      mManager->startUpdatingMap(iMessage->map_id);
      break;
    default:
      break;
    }
  }

  void onMapParams(const lcm::ReceiveBuffer* iBuf,
                   const std::string& iChannel,
                   const drc::map_params_t* iMessage) {
    LocalMap::Spec spec;
    spec.mPointBufferSize = iMessage->buffer_size;
    spec.mActive = true;
    spec.mBoundMin = Eigen::Vector3f(iMessage->bound_min[0],
                                     iMessage->bound_min[1],
                                     iMessage->bound_min[2]);
    spec.mBoundMax = Eigen::Vector3f(iMessage->bound_max[0],
                                     iMessage->bound_max[1],
                                     iMessage->bound_max[2]);
    spec.mOctreeResolution = iMessage->resolution;
    spec.mPointBufferSize = iMessage->buffer_size;
    int id = mManager->createMap(spec);
  }


  void addWorker(const drc::map_request_t& iRequest) {
    WorkerMap::const_iterator item = mWorkers.find(iRequest.view_id);
    if (item != mWorkers.end()) {
      if ((iRequest.type == drc::map_request_t::NONE) ||
          (!iRequest.active)) {
        std::cout << "Stopping view " << iRequest.view_id << std::endl;
        item->second->stop();
      }
      else if (!item->second->mActive) {
        item->second->start();
      }
    }
    else {
      Worker::Ptr worker(new Worker());
      worker->mActive = false;
      worker->mLcm = mLcm;
      worker->mManager = mManager;
      worker->mRequest = iRequest;
      mWorkers[iRequest.view_id] = worker;
      worker->start();
    }
  }
};

class DataConsumer {
public:
  DataConsumer(State* iState) {
    mState = iState;
  }

  void operator()() {
    while(true) {
      maps::PointSet data;
      if (mState->mSensorDataReceiver->waitForData(data)) {
        mState->mManager->addData(data);
      }
    }
  }

protected:
  State* mState;
};

class CatalogSender {
public:
  CatalogSender(State* iState) {
    mState = iState;
  }

  void operator()() {
    while(true) {
      std::cout << "sending catalog" << std::endl;
      drc::map_catalog_t catalog;
      catalog.utime = drc::Clock::instance()->getCurrentTime();
      catalog.views.reserve(mState->mWorkers.size());
      WorkerMap::const_iterator iter = mState->mWorkers.begin();
      for (; iter != mState->mWorkers.end(); ++iter) {
        if (iter->second->mActive) {
          catalog.views.push_back(iter->second->mRequest);
        }
      }
      catalog.num_views = catalog.views.size();
      mState->mLcm->publish("MAP_CATALOG", &catalog);

      // wait to send next catalog
      boost::asio::io_service service;
      boost::asio::deadline_timer timer(service);
      if (mState->mCatalogPublishPeriod > 0) {
        int milli = mState->mCatalogPublishPeriod*1000;
        timer.expires_from_now(boost::posix_time::milliseconds(milli));
        timer.wait();
      }
    }
  }

protected:
  State* mState;
};

int main(const int iArgc, const char** iArgv) {

  // instantiate state object
  State state;

  // parse arguments
  string laserChannel = "ROTATING_SCAN";
  float publishPeriod = 3;
  float defaultResolution = 0.1;
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(laserChannel, "l", "laser_channel",
          "laser channel to use in map creation");
  opt.add(publishPeriod, "p", "publish_period",
          "interval between map publications, in s");
  opt.add(defaultResolution, "r", "resolution",
          "resolution of default contextual map, in m");
  opt.add(state.mCatalogPublishPeriod, "c", "catalog",
          "interval between catalog publications, in s");
  opt.parse();
  state.mSensorDataReceiver->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");

  // set up remaining parameters
  state.mSensorDataReceiver->setMaxBufferSize(100);
  LocalMap::Spec spec;
  spec.mPointBufferSize = 5000;
  spec.mActive = true;
  spec.mBoundMin = Eigen::Vector3f(-5,-5,-5);
  spec.mBoundMax = Eigen::Vector3f(5,5,5);
  spec.mOctreeResolution = 0.1; // TODO
  int id = state.mManager->createMap(spec);
  state.mRequestSubscription =
    state.mLcm->subscribe("MAP_REQUEST", &State::onRequest, &state);
  state.mRequestSubscription =
    state.mLcm->subscribe("MAP_COMMAND", &State::onMapCommand, &state);
  state.mRequestSubscription =
    state.mLcm->subscribe("MAP_PARAMS", &State::onMapParams, &state);

  // start running data receiver
  BotParam* theParam =
    bot_param_new_from_server(state.mLcm->getUnderlyingLCM(), 0);
  state.mSensorDataReceiver->setBotParam(theParam);
  state.mSensorDataReceiver->start();

  // start consuming data
  DataConsumer consumer(&state);
  boost::thread consumerThread(consumer);

  // start sending catalog
  CatalogSender catalogSender(&state);
  boost::thread catalogThread(catalogSender);

  // start publishing data
  drc::map_request_t request;
  request.map_id = 1;
  request.view_id = 1;
  request.type = drc::map_request_t::OCTREE;
  request.resolution = defaultResolution;
  request.frequency = 1.0/publishPeriod;
  request.time_min = -1;
  request.time_max = -1;
  request.num_clip_planes = 0;
  state.addWorker(request);

  // main lcm loop
  while (0 == state.mLcm->handle());

  // join pending threads
  catalogThread.join();
  consumerThread.join();

  return 0;
}
