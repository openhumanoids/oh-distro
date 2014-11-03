#include <unordered_map>
#include <thread>
#include <chrono>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>
#include <lcmtypes/drc/map_command_t.hpp>
#include <lcmtypes/drc/map_params_t.hpp>
#include <lcmtypes/drc/map_catalog_t.hpp>
#include <lcmtypes/drc/data_request_t.hpp>

#include <maps/MapManager.hpp>
#include <maps/PointCloudView.hpp>
#include <maps/OctreeView.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/DepthImage.hpp>
#include <maps/LocalMap.hpp>
#include <maps/SensorDataReceiver.hpp>
#include <maps/DataBlob.hpp>
#include <maps/LcmTranslator.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/Utils.hpp>
#include <maps/Collector.hpp>
#include <maps/PointDataBuffer.hpp>

#include <drc_utils/PointerUtils.hpp>
#include <drc_utils/Clock.hpp>
#include <ConciseArgs>

#include <bot_core/timestamp.h>

#include "LidarFilters.hpp"
#include "FusedDepthHandler.hpp"
#include "StereoHandler.hpp"

using namespace std;
using namespace maps;

// a class to filter lidar points that hit the robot's torso
class TorsoFilter : public LocalMap::Filter {
public:
  TorsoFilter(const BotWrapper::Ptr& iBotWrapper) {
    mBotWrapper = iBotWrapper;
  }

  void operator()(maps::PointSet& ioPoints) {
    // compute scan to torso transform
    Eigen::Isometry3f torsoToLocal;
    mBotWrapper->getTransform("utorso","local",torsoToLocal);
    Eigen::Isometry3f scanToLocal = Utils::getPose(*ioPoints.mCloud);
    Eigen::Isometry3f scanToTorso = torsoToLocal.inverse()*scanToLocal;

    // clip points against torso
    int numPoints = ioPoints.mCloud->size();
    std::vector<int> goodIndices;
    goodIndices.reserve(numPoints);
    for (int i = 0; i < numPoints; ++i) {
      const maps::PointType& pt = (*ioPoints.mCloud)[i];
      Eigen::Vector3f p = scanToTorso*Eigen::Vector3f(pt.x,pt.y,pt.z);
      // NOTE: these bounds were taken from urdf 2013-12-09
      bool bad = ((p[0] < 0.3) && (p[0] > -0.2) && (fabs(p[1]) < 0.25) &&
                  (p[2] < 0.75) && (p[2] > 0));
      if (!bad) goodIndices.push_back(i);
    }

    // create new points
    maps::PointCloud::Ptr cloud(new maps::PointCloud());
    cloud->resize(goodIndices.size());
    cloud->width = goodIndices.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->sensor_origin_ = ioPoints.mCloud->sensor_origin_;
    cloud->sensor_orientation_ = ioPoints.mCloud->sensor_orientation_;
    for (int i = 0; i < goodIndices.size(); ++i) {
      (*cloud)[i] = (*ioPoints.mCloud)[goodIndices[i]];
    }
    ioPoints.mCloud = cloud;
  }
protected:
  BotWrapper::Ptr mBotWrapper;
};

// a class to filter lidar points that are close to the ground
class GroundFilter : public LocalMap::Filter {
public:
  GroundFilter(const BotWrapper::Ptr& iBotWrapper) {
    mBotWrapper = iBotWrapper;
    mActive = true;
  }

  void operator()(maps::PointSet& ioPoints) {
    if (!mActive) return;

    // transform ground plane to scan coords
    Eigen::Isometry3f groundToLocal;
    mBotWrapper->getTransform("ground","local",groundToLocal);
    Eigen::Isometry3f scanToLocal = Utils::getPose(*ioPoints.mCloud);
    Eigen::Isometry3f scanToGround = groundToLocal.inverse()*scanToLocal;
    Eigen::Matrix4f groundPlaneToScan = scanToGround.matrix().transpose();
    const float kDistanceThresh = 0.1;
    Eigen::Vector4f plane = groundPlaneToScan*Eigen::Vector4f(0,0,1,0);

    // include only points higher than threshold above ground
    int numPoints = ioPoints.mCloud->size();
    std::vector<int> goodIndices;
    goodIndices.reserve(numPoints);
    for (int i = 0; i < numPoints; ++i) {
      const maps::PointType& pt = (*ioPoints.mCloud)[i];
      float dist = plane[0]*pt.x + plane[1]*pt.y + plane[2]*pt.z + plane[3];
      if (dist > kDistanceThresh) goodIndices.push_back(i);
    }

    // create new points
    maps::PointCloud::Ptr cloud(new maps::PointCloud());
    cloud->resize(goodIndices.size());
    cloud->width = goodIndices.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->sensor_origin_ = ioPoints.mCloud->sensor_origin_;
    cloud->sensor_orientation_ = ioPoints.mCloud->sensor_orientation_;
    for (int i = 0; i < goodIndices.size(); ++i) {
      (*cloud)[i] = (*ioPoints.mCloud)[goodIndices[i]];
    }
    ioPoints.mCloud = cloud;
  }
protected:
  BotWrapper::Ptr mBotWrapper;
  bool mActive;
};


class State;

struct ViewWorker {
  typedef std::shared_ptr<ViewWorker> Ptr;

  BotWrapper::Ptr mBotWrapper;
  bool mActive;
  drc::map_request_t mRequest;
  std::shared_ptr<Collector> mCollector;
  std::shared_ptr<StereoHandler> mStereoHandlerHead;
  std::shared_ptr<FusedDepthHandler> mFusedDepthHandler;
  std::thread mThread;
  Eigen::Isometry3f mInitialPose;

  ~ViewWorker() {
    stop();
  }

  void start() {
    if (mActive) return;
    if (mThread.joinable()) mThread.join();
    mActive = true;
    mThread = std::thread(std::ref(*this));
  }

  void stop() {
    mActive = false;
    if (mThread.joinable()) mThread.join();
  }

  void operator()() {
    mActive = true;
    while (mActive) {

      // check for cancel
      if (mRequest.frequency < 0) {
        mActive = false;
        break;
      }

      int64_t checkpointStart = bot_timestamp_now();
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

      ViewBase::Spec spec;
      LcmTranslator::fromLcm(mRequest, spec);

      // TODO: HACK for stereo data; need to think about cleaner approach
      if (mRequest.view_id == drc::data_request_t::STEREO_MAP_HEAD) {
        DepthImageView::Ptr view;
        switch (mRequest.view_id) {
        case drc::data_request_t::STEREO_MAP_HEAD:
          if (mStereoHandlerHead != NULL) {
            view = mStereoHandlerHead->getDepthImageView(spec.mClipPlanes);
          }
          break;
        default: break;
        }
        if (view != NULL) {
          view->setId(mRequest.view_id);
          drc::map_image_t msg;
          LcmTranslator::toLcm(*view, msg);
          msg.utime = drc::Clock::instance()->getCurrentTime();
          msg.map_id = mRequest.map_id;
          msg.blob.utime = msg.utime;
          std::string chan =
            mRequest.channel.size()>0 ? mRequest.channel : "MAP_DEPTH";
          lcm->publish(chan, &msg);
          std::cout << "Sent stereo image on " << chan << " at " <<
            msg.blob.num_bytes << " bytes (view " << view->getId() <<
            ")" << std::endl;
        }
        else {
          std::cout << "Could not send stereo image" << std::endl;
        }
      }

      // fused depth
      else if ((mRequest.view_id == drc::data_request_t::FUSED_DEPTH) ||
               (mRequest.view_id == drc::data_request_t::FUSED_HEIGHT) ||
               (mRequest.view_id == drc::data_request_t::STEREO_HEIGHT)) {
        DepthImageView::Ptr view;
        if (mRequest.view_id == drc::data_request_t::FUSED_DEPTH) {
          view = mFusedDepthHandler->getLatest();
        }
        else if (mRequest.view_id == drc::data_request_t::FUSED_HEIGHT) {
          view = mFusedDepthHandler->getLatest(mRequest);
        }
        else if (mRequest.view_id == drc::data_request_t::STEREO_HEIGHT) {
          view = mStereoHandlerHead->getDepthImageView(mRequest);
        }
        if (view == NULL) {
          std::cout << "No view available" << std::endl;
        }
        else {
          view->setId(mRequest.view_id);
          if ((mRequest.view_id == drc::data_request_t::FUSED_HEIGHT) ||
              (mRequest.view_id == drc::data_request_t::STEREO_HEIGHT)) {
            view->setId(drc::data_request_t::HEIGHT_MAP_SCENE);
          }
          drc::map_image_t msg;
          LcmTranslator::toLcm(*view, msg);
          msg.utime = drc::Clock::instance()->getCurrentTime();
          msg.map_id = mRequest.map_id;
          msg.blob.utime = msg.utime;
          std::string chan =
            mRequest.channel.size()>0 ? mRequest.channel : "MAP_DEPTH";
          lcm->publish(chan, &msg);
          std::cout << "Sent stereo depth image on " << chan << " at " <<
            msg.blob.num_bytes << " bytes (view " << view->getId() <<
            ")" << std::endl;
        }
      }

      // all other types
      else if (localMap != NULL) {
        // do not send if there is not enough data
        // TODO: should make this cleaner; for now, 3 seconds
        int64_t timeMin = localMap->getPointData()->getTimeMin();
        int64_t timeMax = localMap->getPointData()->getTimeMax();
        if ((timeMax-timeMin) > 3000000) {

          LocalMap::SpaceTimeBounds bounds;

          // temporal bounds
          int64_t curTime = drc::Clock::instance()->getCurrentTime();
          const double kPi = acos(-1);
          switch (spec.mTimeMode) {
          case ViewBase::TimeModeRelative:
            bounds.mTimeMin = spec.mTimeMin+curTime;
            bounds.mTimeMax = spec.mTimeMax+curTime;
            break;
          case ViewBase::TimeModeRollAngleAbsolute:
            mCollector->getLatestSwath(spec.mTimeMin*kPi/180,
                                       spec.mTimeMax*kPi/180,
                                       bounds.mTimeMin, bounds.mTimeMax);
            break;
          case ViewBase::TimeModeRollAngleRelative:
            mCollector->getLatestSwath(spec.mTimeMin*kPi/180,
                                       spec.mTimeMax*kPi/180,
                                       bounds.mTimeMin, bounds.mTimeMax, true);
            break;
          case ViewBase::TimeModeAbsolute:
          default:
            bounds.mTimeMin = spec.mTimeMin;
            bounds.mTimeMax = spec.mTimeMax;
            break;
          }

          // spatial bounds
          bounds.mPlanes = spec.mClipPlanes;
          Eigen::Isometry3f headToLocal = Eigen::Isometry3f::Identity();
          Eigen::Isometry3f torsoToLocal = Eigen::Isometry3f::Identity();
          if (spec.mRelativeLocation) {
            if (mBotWrapper->getTransform("utorso", "local",
                                          torsoToLocal, curTime) &&
                mBotWrapper->getTransform("head", "local",
                                          headToLocal, curTime)) {
              float theta = atan2(torsoToLocal(1,0), torsoToLocal(0,0));
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
            std::string chan =
              mRequest.channel.size()>0 ? mRequest.channel : "MAP_OCTREE";
            lcm->publish(chan, &octMsg);
            std::cout << "Sent octree on " << chan << " at " <<
              octMsg.num_bytes << " bytes (view " << octree->getId() <<
              ")" << std::endl;
          }

          // get and publish point cloud
          else if (mRequest.type == drc::map_request_t::POINT_CLOUD) {
            PointCloudView::Ptr cloud =
              localMap->getAsPointCloud(mRequest.resolution, bounds);
            cloud->setId(mRequest.view_id);
            drc::map_cloud_t msgCloud;
            LcmTranslator::toLcm(*cloud, msgCloud, mRequest.quantization_max);
            msgCloud.utime = drc::Clock::instance()->getCurrentTime();
            msgCloud.map_id = localMap->getId();
            msgCloud.blob.utime = msgCloud.utime;
            std::string chan =
              mRequest.channel.size()>0 ? mRequest.channel : "MAP_CLOUD";
            lcm->publish(chan, &msgCloud);
            std::cout << "Sent point cloud on " << chan << " at " <<
              msgCloud.blob.num_bytes << " bytes (view " << cloud->getId() <<
              ")" << std::endl;
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
            DepthImage::AccumulationMethod accumMethod =
              (DepthImage::AccumulationMethod)spec.mAccumulationMethod;
            DepthImageView::Ptr image =
              localMap->getAsDepthImage(mRequest.width, mRequest.height,
                                        projector, accumMethod, bounds);
            image->setId(mRequest.view_id);
            drc::map_image_t msgImg;
            LcmTranslator::toLcm(*image, msgImg, mRequest.quantization_max);
            msgImg.utime = drc::Clock::instance()->getCurrentTime();
            msgImg.map_id = localMap->getId();
            msgImg.blob.utime = msgImg.utime;
            std::string chan =
              mRequest.channel.size()>0 ? mRequest.channel : "MAP_DEPTH";
            lcm->publish(chan, &msgImg);
            std::cout << "Sent depth image on " << chan << " at " <<
              msgImg.blob.num_bytes << " bytes (view " << image->getId() <<
              ")" << std::endl;
          }
        }
      }

      // one-shot request has 0 frequency
      if (fabs(mRequest.frequency) < 1e-6) {
        mActive = false;
        break;
      }

      // wait for timer expiry
      int elapsedMs = (bot_timestamp_now() - checkpointStart)/1000;
      int periodMs = 1000/mRequest.frequency;
      int sleepMs = periodMs-elapsedMs;
      if (sleepMs > 0) {
        std::this_thread::sleep_for
          (std::chrono::milliseconds(periodMs-elapsedMs));
      }
    }
  }
};

typedef std::unordered_map<int64_t,ViewWorker::Ptr> ViewWorkerMap;

class State {
public:
  BotWrapper::Ptr mBotWrapper;
  std::shared_ptr<Collector> mCollector;
  ViewWorkerMap mViewWorkers;
  std::shared_ptr<StereoHandler> mStereoHandlerHead;
  std::shared_ptr<FusedDepthHandler> mFusedDepthHandler;

  lcm::Subscription* mRequestSubscription;
  lcm::Subscription* mMapParamsSubscription;
  lcm::Subscription* mMapCommandSubscription;
  lcm::Subscription* mCatalogTriggerSubscription;

  float mCatalogPublishPeriod;
  std::shared_ptr<maps::GroundFilter> mGroundFilter;

  State() {
    mBotWrapper.reset(new BotWrapper());
    while (mBotWrapper->getBotParam() == NULL) {
      std::cout << "Couldn't get BotParam; trying again..." << std::endl;
      mBotWrapper->set(mBotWrapper->getLcm());
    }
    drc::Clock::instance()->setLcm(mBotWrapper->getLcm());
    drc::Clock::instance()->setVerbose(false);
    mCollector.reset(new Collector());
    mCollector->setBotWrapper(mBotWrapper);
    mStereoHandlerHead.reset(new StereoHandler(mBotWrapper, "CAMERA"));
    if (!mStereoHandlerHead->isGood()) mStereoHandlerHead.reset();
    mFusedDepthHandler.reset(new FusedDepthHandler(mBotWrapper));
    // TODO: can we avoid hard-coding these?
    mFusedDepthHandler->setCameraChannel("CAMERA_LEFT");
    mFusedDepthHandler->setDepthChannel("CAMERA_FUSED");
    mFusedDepthHandler->start();
    mRequestSubscription = NULL;
    mMapParamsSubscription = NULL;
    mMapCommandSubscription = NULL;
    mCatalogTriggerSubscription = NULL;
    mCatalogPublishPeriod = 0;
  }

  ~State() {
    auto lcm = mBotWrapper->getLcm();
    lcm->unsubscribe(mRequestSubscription);
    lcm->unsubscribe(mMapParamsSubscription);
    lcm->unsubscribe(mMapCommandSubscription);
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
      worker->mStereoHandlerHead = mStereoHandlerHead;
      worker->mFusedDepthHandler = mFusedDepthHandler;
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
      int milli = mState->mCatalogPublishPeriod*1000;
      std::this_thread::sleep_for(std::chrono::milliseconds(milli));
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
  state.mCollector->bind(laserChannel, 1);

  // this channel is for original scan messages
  auto pos = laserChannel.find("_FREE");
  if (pos != std::string::npos) {
    std::string rawChannel = laserChannel.substr(0,pos);
    state.mCollector->getDataReceiver()->
      addChannel(rawChannel, SensorDataReceiver::SensorTypePlanarLidar,
                 rawChannel, "local");
    state.mCollector->bind(rawChannel, 2);
    state.mCollector->bind(rawChannel, 3);
  }

  // add maps
  LocalMap::Spec mapSpec;
  mapSpec.mId = 1;
  mapSpec.mPointBufferSize = 5000;
  mapSpec.mActive = true;
  mapSpec.mResolution = defaultResolution;
  state.mCollector->getMapManager()->createMap(mapSpec);
  mapSpec.mId = 2;
  state.mCollector->getMapManager()->createMap(mapSpec);

  // add filtered map
  mapSpec.mId = 3;
  state.mCollector->getMapManager()->createMap(mapSpec);
  LocalMap::Ptr localMap =
    state.mCollector->getMapManager()->getMap(mapSpec.mId);
  LocalMap::Filter::Ptr rangeFilter(new LocalMap::RangeFilter());
  std::static_pointer_cast<LocalMap::RangeFilter>(rangeFilter)
    ->setValidRanges(0.1, 3.0);
  LocalMap::Filter::Ptr diffFilter(new LocalMap::RangeDiffFilter());
  std::static_pointer_cast<LocalMap::RangeDiffFilter>(diffFilter)
    ->set(0.03, 2.0);
  LocalMap::Filter::Ptr torsoFilter(new maps::TorsoFilter(state.mBotWrapper));
  state.mGroundFilter.reset(new maps::GroundFilter(state.mBotWrapper));
  localMap->addFilter(rangeFilter);
  localMap->addFilter(diffFilter);
  localMap->addFilter(torsoFilter);
  localMap->addFilter(state.mGroundFilter);

  // set up remaining parameters
  state.mRequestSubscription =
    lcm->subscribe("MAP_REQUEST", &State::onRequest, &state);
  state.mMapCommandSubscription =
    lcm->subscribe("MAP_COMMAND", &State::onMapCommand, &state);
  state.mMapParamsSubscription =
    lcm->subscribe("MAP_PARAMS", &State::onMapParams, &state);
  state.mCatalogTriggerSubscription =
    lcm->subscribe("TRIGGER_MAP_CATALOG", &State::onCatalogTrigger, &state);

  // start running data receiver
  state.mCollector->start();

  // start sending catalog
  CatalogSender catalogSender(&state);
  std::thread catalogThread(catalogSender);

  // start publishing data
  ViewBase::Spec viewSpec;
  viewSpec.mMapId = 1;  // TODO: could make this -1
  viewSpec.mViewId = 1;
  viewSpec.mType = ViewBase::TypeOctree;
  viewSpec.mResolution = defaultResolution;
  viewSpec.mFrequency = 1.0/publishPeriod;
  viewSpec.mTimeMin = viewSpec.mTimeMax = -1;
  viewSpec.mTimeMode = ViewBase::TimeModeAbsolute;
  if (timeWindowSeconds > 0) {
    viewSpec.mTimeMin = -timeWindowSeconds*1e6;
    viewSpec.mTimeMax = 0;
    viewSpec.mTimeMode = ViewBase::TimeModeRelative;
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
  if (catalogThread.joinable()) catalogThread.join();

  return 0;
}
