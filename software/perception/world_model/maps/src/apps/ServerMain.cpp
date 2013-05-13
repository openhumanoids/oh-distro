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
#include <maps/DepthImage.hpp>
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

#include <stereo-bm/stereo-bm.hpp>
#include <bot_param/param_util.h>

using namespace std;
using namespace maps;

class State;

struct StereoHandler {
  BotWrapper::Ptr mBotWrapper;
  bot_core::image_t mLatestImage;
  boost::shared_ptr<StereoB> mStereoMatcher;
  BotCamTrans* mCamTrans;
  Eigen::Matrix3f mCalibMatrix;
  std::string mCameraFrame;
  float mDisparityFactor;

  StereoHandler(const BotWrapper::Ptr& iBotWrapper) {
    mBotWrapper = iBotWrapper;
    auto theLcm = mBotWrapper->getLcm();
    mStereoMatcher.reset(new StereoB(theLcm));
    mStereoMatcher->setScale(1.0);
    mLatestImage.size = 0;

    std::string cameraName = "CAMERALEFT";
    mCamTrans = bot_param_get_new_camtrans
      (mBotWrapper->getBotParam(), cameraName.c_str());
    double k00 = bot_camtrans_get_focal_length_x(mCamTrans);
    double k11 = bot_camtrans_get_focal_length_y(mCamTrans);
    double k01 = bot_camtrans_get_skew(mCamTrans);
    double k02 = bot_camtrans_get_principal_x(mCamTrans);
    double k12 = bot_camtrans_get_principal_y(mCamTrans);
    mCalibMatrix << k00,k01,k02, 0,k11,k12, 0,0,1;
    std::string key("cameras.");
    key += (cameraName + ".coord_frame");
    char* val = NULL;
    if (bot_param_get_str(mBotWrapper->getBotParam(),key.c_str(),&val) == 0) {
      mCameraFrame = val;
      free(val);
    }

    // TODO: can derive the 7cm baseline from camera config
    double baseline = 0.07;
    mDisparityFactor = 1/k00/baseline;

    theLcm->subscribe("CAMERA", &StereoHandler::onImage, this);
  }

  ~StereoHandler() {
    bot_camtrans_destroy(mCamTrans);
  }

  void onImage(const lcm::ReceiveBuffer* iBuffer, const std::string& iChannel,
               const bot_core::image_t* iMessage) {
    mLatestImage = *iMessage;
  }

  DepthImageView::Ptr
  getDepthImageView(const std::vector<Eigen::Vector4f>& iBoundPlanes) {
    DepthImageView::Ptr view;
    if (mLatestImage.size == 0) {
      return view;
    }
    view.reset(new DepthImageView());

    // split images
    int w(mLatestImage.width), h(mLatestImage.height/2);
    cv::Mat img(2*h, w, CV_8UC1, mLatestImage.data.data());
    cv::Mat leftImage = img.rowRange(0, h).clone();
    cv::Mat rightImage = img.rowRange(h, 2*h).clone();

    // compute disparity
    mStereoMatcher->doStereoB(leftImage, rightImage);
    cv::Mat disparityMat;
    cv::Mat(h,w,CV_16UC1,mStereoMatcher->getDisparity()).
      convertTo(disparityMat, CV_32F, 1.0f/16);

    // project bound polyhedron onto camera
    std::vector<Eigen::Vector3f> vertices;
    std::vector<std::vector<int> > faces;
    Utils::polyhedronFromPlanes(iBoundPlanes, vertices, faces);
    Eigen::Isometry3f localToCamera;
    mBotWrapper->getTransform("local", mCameraFrame, localToCamera,
                              mLatestImage.utime);
    Eigen::Vector2i minPt(1000000,1000000), maxPt(-1000000,-1000000);
    for (size_t i = 0; i < vertices.size(); ++i) {
      Eigen::Vector3f pt = localToCamera*vertices[i];
      double inPt[] = { pt[0], pt[1], pt[2] };
      double proj[3];
      bot_camtrans_project_point(mCamTrans, inPt, proj);
      minPt[0] = std::min(minPt[0], (int)floor(proj[0]));
      minPt[1] = std::min(minPt[1], (int)floor(proj[1]));
      maxPt[0] = std::max(maxPt[0], (int)ceil(proj[0]));
      maxPt[1] = std::max(maxPt[1], (int)ceil(proj[1]));
    }
    minPt[0] = std::max(0, minPt[0]);
    minPt[1] = std::max(0, minPt[1]);
    maxPt[0] = std::min(w-1, maxPt[0]);
    maxPt[1] = std::min(h-1, maxPt[1]);

    // crop and copy
    // TODO: set to 0 points that are outside clip polyhedron
    Eigen::Vector2i newSize = maxPt - minPt + Eigen::Vector2i(1,1);
    cv::Rect bounds(minPt[0], minPt[1], newSize[0], newSize[1]);
    cv::Mat disparitySub = disparityMat(bounds);
    std::vector<float> dispData(newSize[0]*newSize[1]);
    float* outPtr = &dispData[0];
    for (int i = 0; i < newSize[1]; ++i) {
      float* inPtr = disparitySub.ptr<float>(i);
      for (int j = 0; j < newSize[0]; ++j, ++inPtr, ++outPtr) {
        float val = *inPtr;
        *outPtr = (val > 4000) ? 0 : val*mDisparityFactor;
      }
    }
    Eigen::Matrix3f calib = mCalibMatrix;
    calib(0,2) -= minPt[0];
    calib(1,2) -= minPt[1];

    // form output image and view
    DepthImage depthImage;
    depthImage.setSize(newSize[0], newSize[1]);
    depthImage.setOrthographic(false);
    depthImage.setPose(localToCamera.inverse());
    depthImage.setCalib(calib);
    depthImage.setData(dispData, DepthImage::TypeDisparity);
    view->set(depthImage);
    return view;
  }
};

struct ViewWorker {
  typedef boost::shared_ptr<ViewWorker> Ptr;

  BotWrapper::Ptr mBotWrapper;
  bool mActive;
  drc::map_request_t mRequest;
  boost::shared_ptr<Collector> mCollector;
  boost::shared_ptr<StereoHandler> mStereoHandler;
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

      ViewBase::Spec spec;
      LcmTranslator::fromLcm(mRequest, spec);

      // TODO: HACK for stereo data; need to think about cleaner approach
      if (mRequest.map_id == 1111) {
        DepthImageView::Ptr view =
          mStereoHandler->getDepthImageView(spec.mClipPlanes);
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
      }

      if (localMap != NULL) {

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
          LcmTranslator::toLcm(*cloud, msgCloud);
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
          DepthImageView::Ptr image =
            localMap->getAsDepthImage(mRequest.width, mRequest.height,
                                      projector, bounds);
          image->setId(mRequest.view_id);
          drc::map_image_t msgImg;
          LcmTranslator::toLcm(*image, msgImg);
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

      // one-shot request has 0 frequency
      if (fabs(mRequest.frequency) < 1e-6) {
        mActive = false;
        break;
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
  boost::shared_ptr<StereoHandler> mStereoHandler;

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
    mStereoHandler.reset(new StereoHandler(mBotWrapper));
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
      worker->mStereoHandler = mStereoHandler;
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
