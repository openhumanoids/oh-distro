#include <unordered_map>

#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>
#include <lcmtypes/drc/map_command_t.hpp>
#include <lcmtypes/drc/map_params_t.hpp>
#include <lcmtypes/drc/map_catalog_t.hpp>
#include <lcmtypes/drc/map_macro_t.hpp>
#include <lcmtypes/drc/twist_timed_t.hpp>

#include <maps/MapManager.hpp>
#include <maps/PointCloudView.hpp>
#include <maps/OctreeView.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/LocalMap.hpp>
#include <maps/SensorDataReceiver.hpp>
#include <maps/DataBlob.hpp>
#include <maps/LcmTranslator.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/Utils.hpp>
#include <maps/Collector.hpp>

#include <drc_utils/Clock.hpp>
#include <pcl/common/transforms.h>
#include <ConciseArgs>
#include <octomap/octomap.h>

using namespace std;
using namespace maps;

class State;

struct ViewWorker {
  typedef boost::shared_ptr<ViewWorker> Ptr;

  BotWrapper::Ptr mBotWrapper;
  bool mActive;
  drc::map_request_t mRequest;
  boost::shared_ptr<Collector> mCollector;
  boost::thread mThread;
  Eigen::Isometry3f mInitialPose;

  ~ViewWorker() {
    stop();
  }

  void start() {
    if (mActive) return;
    if (mThread.joinable()) mThread.join();
    mActive = true;
    mThread = boost::thread(boost::ref(*this));
  }

  void stop() {
    mActive = false;
    mThread.join();
  }

  void operator()() {
    mActive = true;
    while (mActive) {
      auto lcm = mBotWrapper->getLcm();
      // get map
      LocalMap::Ptr localMap;
      auto manager = mCollector->getMapManager();
      if (mRequest.map_id <= 0) {
        vector<int64_t> ids = manager->getAllMapIds(true);
        if (ids.size() > 0) {
          localMap = manager->getMap(ids.back());
        }
      }
      else {
        localMap = manager->getMap(mRequest.map_id);
      }
      if (localMap != NULL) {

        ViewBase::Spec spec;
        LcmTranslator::fromLcm(mRequest, spec);

        // get bounds
        LocalMap::SpaceTimeBounds bounds;
        bounds.mTimeMin = spec.mTimeMin;
        bounds.mTimeMax = spec.mTimeMax;
        bounds.mPlanes = spec.mClipPlanes;

        // transform bounds if necessary
        int64_t curTime = drc::Clock::instance()->getCurrentTime();
        if (spec.mRelativeTime) {
          bounds.mTimeMin += curTime;
          bounds.mTimeMax += curTime;
        }
        Eigen::Isometry3f headToLocal = Eigen::Isometry3f::Identity();
        if (spec.mRelativeLocation) {
          if (mBotWrapper->getTransform("head", "local",
                                        headToLocal, curTime)) {
            float theta = atan2(headToLocal(1,0), headToLocal(0,0));
            Eigen::Matrix3f rotation;
            rotation = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());
            headToLocal.linear() = rotation;
            Eigen::Matrix4f planeTransform =
              (headToLocal.inverse()).matrix().transpose();
            for (int i = 0; i < bounds.mPlanes.size(); ++i) {
              bounds.mPlanes[i] = planeTransform*bounds.mPlanes[i];
            }
          }
        }

        // get and publish octree
        if (mRequest.type == drc::map_request_t::OCTREE) {
          OctreeView::Ptr octree =
            localMap->getAsOctree(mRequest.resolution, false,
                                  Eigen::Vector3f(0,0,0), bounds);
          octree->setId(mRequest.view_id);
          std::cout << "Publishing octree..." << std::endl;
          drc::map_octree_t octMsg;
          LcmTranslator::toLcm(*octree, octMsg);
          octMsg.utime = drc::Clock::instance()->getCurrentTime();
          octMsg.map_id = localMap->getId();
          lcm->publish("MAP_OCTREE", &octMsg);
          std::cout << "Sent octree at " << octMsg.num_bytes <<
            " bytes (view " << octree->getId() << ")" << std::endl;
        }

        // get and publish point cloud
        else if (mRequest.type == drc::map_request_t::POINT_CLOUD) {
          PointCloudView::Ptr cloud =
            localMap->getAsPointCloud(mRequest.resolution, bounds);
          cloud->setId(mRequest.view_id);
          drc::map_cloud_t msgCloud;
          LcmTranslator::toLcm(*cloud, msgCloud);
          msgCloud.utime = drc::Clock::instance()->getCurrentTime();
          msgCloud.map_id = localMap->getId();
          msgCloud.blob.utime = msgCloud.utime;
          lcm->publish("MAP_CLOUD", &msgCloud);
          std::cout << "Sent point cloud at " << msgCloud.blob.num_bytes <<
            " bytes (view " << cloud->getId() << ")" << std::endl;
        }

        // get and publish depth image
        else if (mRequest.type == drc::map_request_t::DEPTH_IMAGE) {
          Eigen::Projective3f projector;
          for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
              projector(i,j) = mRequest.transform[i][j];
            }
          }
          if (spec.mRelativeLocation) {
            projector = projector*headToLocal.inverse();
          }
          DepthImageView::Ptr image =
            localMap->getAsDepthImage(mRequest.width, mRequest.height,
                                      projector, bounds);
          image->setId(mRequest.view_id);
          drc::map_image_t msgImg;
          LcmTranslator::toLcm(*image, msgImg);
          msgImg.utime = drc::Clock::instance()->getCurrentTime();
          msgImg.map_id = localMap->getId();
          msgImg.blob.utime = msgImg.utime;
          lcm->publish("MAP_DEPTH", &msgImg);
          std::cout << "Sent depth image at " << msgImg.blob.num_bytes <<
            " bytes (view " << image->getId() << ")" << std::endl;
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
    }
  }
};

typedef std::unordered_map<int64_t,ViewWorker::Ptr> ViewWorkerMap;

class State {
public:
  BotWrapper::Ptr mBotWrapper;
  boost::shared_ptr<Collector> mCollector;
  ViewWorkerMap mViewWorkers;

  lcm::Subscription* mRequestSubscription;
  lcm::Subscription* mMapParamsSubscription;
  lcm::Subscription* mMapCommandSubscription;
  lcm::Subscription* mMapMacroSubscription;
  lcm::Subscription* mCatalogTriggerSubscription;

  float mCatalogPublishPeriod;

  State() {
    mBotWrapper.reset(new BotWrapper());
    drc::Clock::instance()->setLcm(mBotWrapper->getLcm());
    drc::Clock::instance()->setVerbose(false);
    mCollector.reset(new Collector());
    mCollector->setBotWrapper(mBotWrapper);
    mRequestSubscription = NULL;
    mMapParamsSubscription = NULL;
    mMapCommandSubscription = NULL;
    mMapMacroSubscription = NULL;
    mCatalogTriggerSubscription = NULL;
    mCatalogPublishPeriod = 0;
  }

  ~State() {
    auto lcm = mBotWrapper->getLcm();
    lcm->unsubscribe(mRequestSubscription);
    lcm->unsubscribe(mMapParamsSubscription);
    lcm->unsubscribe(mMapCommandSubscription);
    lcm->unsubscribe(mMapMacroSubscription);
    lcm->unsubscribe(mCatalogTriggerSubscription);
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
    auto manager = mCollector->getMapManager();
    if (id < 0) {
      vector<int64_t> ids = manager->getAllMapIds(true);
      if (ids.size() > 0) {
        manager->getMap(ids.back())->getId();
      }
      else {
        std::cout << "Error: no maps available" << std::endl;
        return;
      }
    }
    switch (iMessage->command) {
    case drc::map_command_t::CLEAR:
      manager->clearMap(id);
      break;
    case drc::map_command_t::STOP:
      manager->stopUpdatingMap(id);
      break;
    case drc::map_command_t::START:
      manager->startUpdatingMap(id);
      break;
    default:
      break;
    }
  }

  void onMapParams(const lcm::ReceiveBuffer* iBuf,
                   const std::string& iChannel,
                   const drc::map_params_t* iMessage) {
    LocalMap::Spec spec;
    LcmTranslator::fromLcm(*iMessage, spec);
    mCollector->getMapManager()->createMap(spec);
  }

  void onMapMacro(const lcm::ReceiveBuffer* iBuf,
                  const std::string& iChannel,
                  const drc::map_macro_t* iMessage) {
    struct MacroWorker {
      State* mState;
      drc::map_macro_t mMacro;

      void operator()() {
        // create single-scan dense map
        auto lcm = mState->mBotWrapper->getLcm();
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
          lcm->publish("ROTATING_SCAN_RATE_CMD", &rate);

          std::cout << "Creating new map..." << std::endl;
          LocalMap::Spec spec;
          spec.mResolution = 0.01;
          int id = mState->mCollector->getMapManager()->createMap(spec);

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
          mState->mCollector->getMapManager()->stopUpdatingMap(id);

          std::cout << "Resetting spindle speed..." << std::endl;
          rate.angular_velocity.x = 2*kPi;
          lcm->publish("ROTATING_SCAN_RATE_CMD", &rate);

          std::cout << "Done creating dense map" << std::endl;
        }
        else if (drc::map_macro_t::GROUND_SCAN_MODE) {
          // TODO: may need to rework this; maybe a single thread for all time
          // that can switch modes if it's interrupted by new macro
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

  void onCatalogTrigger(const lcm::ReceiveBuffer* iBuf,
                        const std::string& iChannel) {
    sendCatalog();
  }

  void addViewWorker(const drc::map_request_t& iRequest) {
    ViewWorkerMap::const_iterator item = mViewWorkers.find(iRequest.view_id);
    if (item != mViewWorkers.end()) {
      if ((iRequest.type == drc::map_request_t::NONE) ||
          (!iRequest.active)) {
        std::cout << "Removing view " << iRequest.view_id << std::endl;
        ViewWorker::Ptr worker = item->second;
        worker->stop();
        mViewWorkers.erase(item);
        worker.reset();
      }
      else {
        item->second->mRequest = iRequest;
        if (!item->second->mActive) {
          item->second->start();
        }
      }
    }
    else {
      ViewWorker::Ptr worker(new ViewWorker());
      worker->mBotWrapper = mBotWrapper;
      worker->mActive = false;
      worker->mCollector = mCollector;
      worker->mRequest = iRequest;
      worker->mInitialPose = Eigen::Isometry3f::Identity();
      mBotWrapper->getTransform("head", "local", worker->mInitialPose,
                                worker->mRequest.utime);
      mViewWorkers[iRequest.view_id] = worker;
      worker->start();
    }
  }

  void sendCatalog() {
    std::cout << "sending catalog" << std::endl;
    drc::map_catalog_t catalog;
    catalog.utime = drc::Clock::instance()->getCurrentTime();
    auto manager = mCollector->getMapManager();
    std::vector<int64_t> mapIds = manager->getAllMapIds();
    catalog.maps.reserve(mapIds.size());
    for (int i = 0; i < mapIds.size(); ++i) {
      LocalMap::Ptr localMap = manager->getMap(mapIds[i]);
      if (localMap != NULL) {
        drc::map_params_t params;
        LcmTranslator::toLcm(localMap->getSpec(), params);
        params.utime = catalog.utime;
        catalog.maps.push_back(params);
      }
    }
    catalog.views.reserve(mViewWorkers.size());
    ViewWorkerMap::const_iterator iter = mViewWorkers.begin();
    for (; iter != mViewWorkers.end(); ++iter) {
      catalog.views.push_back(iter->second->mRequest);
    }
    catalog.num_views = catalog.views.size();
    catalog.num_maps = catalog.maps.size();
    mBotWrapper->getLcm()->publish("MAP_CATALOG", &catalog);
  }
};

class CatalogSender {
public:
  CatalogSender(State* iState) {
    mState = iState;
  }

  void operator()() {
    if (mState->mCatalogPublishPeriod == 0) return;
    while(true) {
      mState->sendCatalog();

      // wait to send next catalog
      boost::asio::io_service service;
      boost::asio::deadline_timer timer(service);
      int milli = mState->mCatalogPublishPeriod*1000;
      timer.expires_from_now(boost::posix_time::milliseconds(milli));
      timer.wait();
    }
  }

protected:
  State* mState;
};

int main(const int iArgc, const char** iArgv) {

  // instantiate state object
  State state;
  auto lcm = state.mBotWrapper->getLcm();

  // parse arguments
  string laserChannel = "SCAN_FREE";
  float publishPeriod = 0;
  float defaultResolution = 0.1;
  float timeWindowSeconds = 0;
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(laserChannel, "l", "laser_channel",
          "laser channel to use in map creation");
  opt.add(publishPeriod, "p", "publish_period",
          "interval between map publications, in s (0 for none)");
  opt.add(defaultResolution, "r", "resolution",
          "resolution of default contextual map, in m");
  opt.add(state.mCatalogPublishPeriod, "c", "catalog",
          "interval between catalog publications, in s");
  opt.add(timeWindowSeconds, "w", "window",
          "time window of default contextual map, in s");
  opt.parse();
  state.mCollector->getDataReceiver()->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");

  // set up remaining parameters
  LocalMap::Spec mapSpec;
  mapSpec.mId = 1;
  mapSpec.mPointBufferSize = 5000;
  mapSpec.mActive = true;
  mapSpec.mBoundMin = Eigen::Vector3f(-1,-1,-1)*1e10;
  mapSpec.mBoundMax = Eigen::Vector3f(1,1,1)*1e10;
  mapSpec.mResolution = defaultResolution;
  state.mCollector->getMapManager()->createMap(mapSpec);
  state.mRequestSubscription =
    lcm->subscribe("MAP_REQUEST", &State::onRequest, &state);
  state.mMapCommandSubscription =
    lcm->subscribe("MAP_COMMAND", &State::onMapCommand, &state);
  state.mMapParamsSubscription =
    lcm->subscribe("MAP_PARAMS", &State::onMapParams, &state);
  state.mMapMacroSubscription =
    lcm->subscribe("MAP_MACRO", &State::onMapMacro, &state);
  state.mCatalogTriggerSubscription =
    lcm->subscribe("TRIGGER_MAP_CATALOG", &State::onCatalogTrigger, &state);

  // start running data receiver
  state.mCollector->start();

  // start sending catalog
  CatalogSender catalogSender(&state);
  boost::thread catalogThread(catalogSender);

  // start publishing data
  ViewBase::Spec viewSpec;
  viewSpec.mMapId = 1;  // TODO: could make this -1
  viewSpec.mViewId = 1;
  viewSpec.mType = ViewBase::TypeOctree;
  viewSpec.mResolution = defaultResolution;
  viewSpec.mFrequency = 1.0/publishPeriod;
  viewSpec.mTimeMin = viewSpec.mTimeMax = -1;
  viewSpec.mRelativeTime = false;
  if (timeWindowSeconds > 0) {
    viewSpec.mTimeMin = -timeWindowSeconds*1e6;
    viewSpec.mTimeMax = 0;
    viewSpec.mRelativeTime = true;
  }
  viewSpec.mClipPlanes = Utils::planesFromBox(Eigen::Vector3f(-5,-5,-5),
                                              Eigen::Vector3f(5,5,5));
  viewSpec.mActive = true;
  viewSpec.mRelativeLocation = true;
  viewSpec.mWidth = viewSpec.mHeight = 0;
  drc::map_request_t request;
  LcmTranslator::toLcm(viewSpec, request);
  request.utime = drc::Clock::instance()->getCurrentTime();
  if (publishPeriod > 0) {
    state.addViewWorker(request);
  }

  // main lcm loop
  while (0 == lcm->handle());

  // join pending threads
  catalogThread.join();

  return 0;
}
