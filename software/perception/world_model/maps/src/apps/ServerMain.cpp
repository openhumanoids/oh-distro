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
#include <lcmtypes/drc/map_macro_t.hpp>
#include <lcmtypes/drc/twist_timed_t.hpp>

#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/SensorDataReceiver.hpp>
#include <maps/DataBlob.hpp>
#include <maps/LcmTranslator.hpp>
#include <maps/BotFramesWrapper.hpp>
#include <maps/Utils.hpp>

#include <drc_utils/Clock.hpp>
#include <pcl/common/transforms.h>
#include <ConciseArgs>
#include <octomap/octomap.h>


using namespace std;
using namespace maps;

class State;

struct ViewWorker {
  typedef boost::shared_ptr<ViewWorker> Ptr;

  bool mActive;
  drc::map_request_t mRequest;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<BotFramesWrapper> mFrames;
  boost::thread mThread;
  Eigen::Isometry3f mInitialPose;

  ~ViewWorker() {
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
      if (mRequest.map_id <= 0) {
        vector<int64_t> ids = mManager->getAllMapIds(true);
        if (ids.size() > 0) {
          localMap = mManager->getMap(ids.back());
        }
      }
      else {
        localMap = mManager->getMap(mRequest.map_id);
      }
      if (localMap != NULL) {

        MapView::Spec spec = LcmTranslator::fromLcm(mRequest);

        // get bounds
        LocalMap::SpaceTimeBounds bounds;
        bounds.mMinTime = spec.mTimeMin;
        bounds.mMaxTime = spec.mTimeMax;
        bounds.mPlanes = spec.mClipPlanes;

        // transform bounds if necessary
        int64_t curTime = drc::Clock::instance()->getCurrentTime();
        if (spec.mRelativeTime) {
          bounds.mMinTime += curTime;
          bounds.mMaxTime += curTime;
        }
        if (spec.mRelativeLocation) {
          for (int i = 0; i < bounds.mPlanes.size(); ++i) {
            Eigen::Vector4f plane = bounds.mPlanes[i];
            Eigen::Vector3f pos;
            Eigen::Quaternionf rot;
            if (mFrames->getTransform("head", "local", curTime, pos, rot)) {
              bounds.mPlanes[i][3] = -pos.dot(plane.head<3>()) + plane[3];
            }
          }
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
          std::cout << "Sent octree at " << octMsg.num_bytes <<
            " bytes" << std::endl;
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

typedef std::unordered_map<int64_t,ViewWorker::Ptr> ViewWorkerMap;

class State {
public:
  boost::shared_ptr<SensorDataReceiver> mSensorDataReceiver;
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<BotFramesWrapper> mFrames;
  ViewWorkerMap mViewWorkers;

  lcm::Subscription* mRequestSubscription;
  lcm::Subscription* mMapParamsSubscription;
  lcm::Subscription* mMapCommandSubscription;
  lcm::Subscription* mMapMacroSubscription;

  float mCatalogPublishPeriod;

  State() {
    mLcm.reset(new lcm::LCM());
    mSensorDataReceiver.reset(new SensorDataReceiver());
    mSensorDataReceiver->setLcm(mLcm);
    mManager.reset(new MapManager());
    mFrames.reset(new BotFramesWrapper());
    mFrames->setLcm(mLcm);
    drc::Clock::instance()->setLcm(mLcm);
    mRequestSubscription = NULL;
    mMapParamsSubscription = NULL;
    mMapCommandSubscription = NULL;
    mMapMacroSubscription = NULL;
    mCatalogPublishPeriod = 10;
  }

  ~State() {
    mLcm->unsubscribe(mRequestSubscription);
    mLcm->unsubscribe(mMapParamsSubscription);
    mLcm->unsubscribe(mMapCommandSubscription);
    mLcm->unsubscribe(mMapMacroSubscription);
  }

  void onRequest(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const drc::map_request_t* iMessage) {
    addViewWorker(*iMessage);
  }

  void onMapCommand(const lcm::ReceiveBuffer* iBuf,
                    const std::string& iChannel,
                    const drc::map_command_t* iMessage) {
    int64_t id = iMessage->map_id;
    if (id < 0) {
      vector<int64_t> ids = mManager->getAllMapIds(true);
      if (ids.size() > 0) {
        mManager->getMap(ids.back())->getId();
      }
      else {
        std::cout << "Error: no maps available" << std::endl;
        return;
      }
    }
    switch (iMessage->command) {
    case drc::map_command_t::CLEAR:
      mManager->clearMap(id);
      break;
    case drc::map_command_t::STOP:
      mManager->stopUpdatingMap(id);
      break;
    case drc::map_command_t::START:
      mManager->startUpdatingMap(id);
      break;
    default:
      break;
    }
  }

  void onMapParams(const lcm::ReceiveBuffer* iBuf,
                   const std::string& iChannel,
                   const drc::map_params_t* iMessage) {
    LocalMap::Spec spec = LcmTranslator::fromLcm(*iMessage);
    int id = mManager->createMap(spec);
  }

  void onMapMacro(const lcm::ReceiveBuffer* iBuf,
                  const std::string& iChannel,
                  const drc::map_macro_t* iMessage) {
    struct MacroWorker {
      State* mState;
      drc::map_macro_t mMacro;
      void operator()() {
        if (mMacro.command == drc::map_macro_t::CREATE_DENSE_MAP) {
          std::cout << "About to create dense map" << std::endl;

          // TODO: could move rate controls into renderer
          std::cout << "Slowing spindle..." << std::endl;
          const double kPi = 4*atan(1);
          drc::twist_timed_t rate;
          rate.utime = drc::Clock::instance()->getCurrentTime();
          rate.angular_velocity.x = 2*kPi/12;
          rate.angular_velocity.y = 0.0;
          rate.angular_velocity.z = 0.0;
          rate.linear_velocity.x = 0.0;
          rate.linear_velocity.y = 0.0;
          rate.linear_velocity.z = 0.0;
          mState->mLcm->publish("ROTATING_SCAN_RATE_CMD", &rate);

          std::cout << "Creating new map..." << std::endl;
          LocalMap::Spec spec;
          spec.mResolution = 0.01;
          int id = mState->mManager->createMap(spec);

          std::cout << "Waiting for accumulation..." << std::endl;
          int64_t baseTime = drc::Clock::instance()->getCurrentTime();
          double minTime = kPi/rate.angular_velocity.x;
          while (true) {
            int64_t curTime = drc::Clock::instance()->getCurrentTime();
            double dt = double(curTime-baseTime)/1e6;
            if (dt > minTime) break;
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
          }

          std::cout << "Stopping accumulation..." << std::endl;
          mState->mManager->stopUpdatingMap(id);

          std::cout << "Resetting spindle speed..." << std::endl;
          rate.angular_velocity.x = 2*kPi;
          mState->mLcm->publish("ROTATING_SCAN_RATE_CMD", &rate);

          std::cout << "Done creating dense map" << std::endl;
        }
        else {
          std::cout << "Invalid macro" << std::endl;
        }
      }
    };
    MacroWorker worker;
    worker.mMacro = *iMessage;
    worker.mState = this;
    boost::thread thread(worker);
  }


  void addViewWorker(const drc::map_request_t& iRequest) {
    ViewWorkerMap::const_iterator item = mViewWorkers.find(iRequest.view_id);
    if (item != mViewWorkers.end()) {
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
      ViewWorker::Ptr worker(new ViewWorker());
      worker->mActive = false;
      worker->mLcm = mLcm;
      worker->mManager = mManager;
      worker->mRequest = iRequest;
      worker->mFrames = mFrames;
      worker->mInitialPose = Eigen::Isometry3f::Identity();
      mFrames->getTransform("head", "local", worker->mRequest.utime,
                            worker->mInitialPose);
      mViewWorkers[iRequest.view_id] = worker;
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
      std::vector<int64_t> mapIds = mState->mManager->getAllMapIds();
      catalog.maps.reserve(mapIds.size());
      for (int i = 0; i < mapIds.size(); ++i) {
        LocalMap::Ptr localMap = mState->mManager->getMap(mapIds[i]);
        if (localMap != NULL) {
          catalog.maps.push_back(LcmTranslator::toLcm(localMap->getSpec()));
        }
      }
      catalog.views.reserve(mState->mViewWorkers.size());
      ViewWorkerMap::const_iterator iter = mState->mViewWorkers.begin();
      for (; iter != mState->mViewWorkers.end(); ++iter) {
        if (iter->second->mActive) {
          catalog.views.push_back(iter->second->mRequest);
        }
      }
      catalog.num_views = catalog.views.size();
      catalog.num_maps = catalog.maps.size();
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
  LocalMap::Spec mapSpec;
  mapSpec.mId = 1;
  mapSpec.mPointBufferSize = 5000;
  mapSpec.mActive = true;
  mapSpec.mBoundMin = Eigen::Vector3f(-1,-1,-1)*1e10;
  mapSpec.mBoundMax = Eigen::Vector3f(1,1,1)*1e10;
  mapSpec.mResolution = defaultResolution;
  int id = state.mManager->createMap(mapSpec);
  state.mRequestSubscription =
    state.mLcm->subscribe("MAP_REQUEST", &State::onRequest, &state);
  state.mRequestSubscription =
    state.mLcm->subscribe("MAP_COMMAND", &State::onMapCommand, &state);
  state.mRequestSubscription =
    state.mLcm->subscribe("MAP_PARAMS", &State::onMapParams, &state);
  state.mRequestSubscription =
    state.mLcm->subscribe("MAP_MACRO", &State::onMapMacro, &state);

  // start running data receiver
  state.mSensorDataReceiver->start();

  // start consuming data
  DataConsumer consumer(&state);
  boost::thread consumerThread(consumer);

  // start sending catalog
  CatalogSender catalogSender(&state);
  boost::thread catalogThread(catalogSender);

  // start publishing data
  MapView::Spec viewSpec;
  viewSpec.mMapId = 1;  // TODO: could make this -1
  viewSpec.mViewId = 1;
  viewSpec.mType = MapView::Spec::TypeOctree;
  viewSpec.mResolution = defaultResolution;
  viewSpec.mFrequency = 1.0/publishPeriod;
  viewSpec.mTimeMin = viewSpec.mTimeMax = -1;
  viewSpec.mClipPlanes = Utils::planesFromBox(Eigen::Vector3f(-5,-5,-5),
                                              Eigen::Vector3f(5,5,5));
  viewSpec.mActive = true;
  viewSpec.mRelativeTime = false;
  viewSpec.mRelativeLocation = true;
  drc::map_request_t request = LcmTranslator::toLcm(viewSpec);
  state.addViewWorker(request);

  // main lcm loop
  while (0 == state.mLcm->handle());

  // join pending threads
  catalogThread.join();
  consumerThread.join();

  return 0;
}
