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
#include <lcmtypes/drc/map_macro_t.hpp>
#include <lcmtypes/drc/data_request_t.hpp>
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
#include <maps/PointDataBuffer.hpp>

#include <drc_utils/PointerUtils.hpp>
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
  std::shared_ptr<StereoB> mStereoMatcher;
  BotCamTrans* mCamTrans;
  Eigen::Matrix3f mCalibMatrix;
  std::string mCameraFrame;
  float mDisparityFactor;

  StereoHandler(const BotWrapper::Ptr& iBotWrapper,
                const std::string& iCameraBaseName) {
    mBotWrapper = iBotWrapper;
    auto lcm = mBotWrapper->getLcm();
    auto boostLcm = drc::PointerUtils::boostPtr(lcm);
    mStereoMatcher.reset(new StereoB(boostLcm));
    mStereoMatcher->setScale(1.0);
    mLatestImage.size = 0;

    std::string cameraName = iCameraBaseName + "LEFT";
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
    double baseline = 0.04;
    if (iCameraBaseName == "CAMERA") baseline = 0.07;
    mDisparityFactor = 1/k00/baseline;

    lcm->subscribe(iCameraBaseName, &StereoHandler::onImage, this);
  }

  ~StereoHandler() {
    bot_camtrans_destroy(mCamTrans);
  }

  void onImage(const lcm::ReceiveBuffer* iBuffer, const std::string& iChannel,
               const bot_core::image_t* iMessage) {
    mLatestImage = *iMessage;
  }

  bool cropPoint(const Eigen::Vector3f& iPoint,
                 const std::vector<Eigen::Vector4f>& iBoundPlanes) {
    for (size_t k = 0; k < iBoundPlanes.size(); ++k) {
      const Eigen::Vector4f& plane = iBoundPlanes[k];
      float dist = plane[0]*iPoint[0] + plane[1]*iPoint[1] + plane[2]*iPoint[2];
      if (dist < -plane[3]) return false;
    }
    return true;
  }

  DepthImageView::Ptr
  getDepthImageView(const std::vector<Eigen::Vector4f>& iBoundPlanes) {
    DepthImageView::Ptr view;
    if (mLatestImage.size == 0) {
      return view;
    }

    // project bound polyhedron onto camera
    int w(mLatestImage.width), h(mLatestImage.height/2);
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
    if ((minPt[0] >= maxPt[0]) || (minPt[1] >= maxPt[1])) {
      std::cout << "Warning: requested volume is outside image" << std::endl;
      return view;
    }

    view.reset(new DepthImageView());

    // uncompress image if necessary
    cv::Mat img;
    if (mLatestImage.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG) {
      img = cv::imdecode(cv::Mat(mLatestImage.data), -1);
    }
    else {
      img = cv::Mat(2*h, w, CV_8UC1, mLatestImage.data.data());
    }

    // split images
    cv::Mat leftImage = img.rowRange(0, h).clone();
    cv::Mat rightImage = img.rowRange(h, 2*h).clone();

    // compute disparity
    mStereoMatcher->doStereoB(leftImage, rightImage);
    cv::Mat disparityMat;
    cv::Mat(h,w,CV_16UC1,mStereoMatcher->getDisparity()).
      convertTo(disparityMat, CV_32F, 1.0f/16);

    // form output depth image
    DepthImage depthImage;
    Eigen::Vector2i newSize = maxPt - minPt + Eigen::Vector2i(1,1);
    depthImage.setSize(newSize[0], newSize[1]);
    depthImage.setOrthographic(false);
    depthImage.setPose(localToCamera.inverse());
    Eigen::Matrix3f calib = mCalibMatrix;
    calib(0,2) -= minPt[0];
    calib(1,2) -= minPt[1];
    depthImage.setCalib(calib);

    // scale down if necessary
    // TODO

    // crop and copy disparity data
    cv::Rect bounds(minPt[0], minPt[1], newSize[0], newSize[1]);
    cv::Mat disparitySub = disparityMat(bounds);
    std::vector<float> dispData(newSize[0]*newSize[1]);
    float* outPtr = &dispData[0];
    for (int i = 0; i < newSize[1]; ++i) {
      float* inPtr = disparitySub.ptr<float>(i);
      for (int j = 0; j < newSize[0]; ++j, ++inPtr, ++outPtr) {
        float dispVal = *inPtr;
        if (dispVal > 4000) {
          *outPtr = 0;
        }
        else {
          Eigen::Vector3f pt(j,i,dispVal*mDisparityFactor);
          Eigen::Vector3f ptLocal =
            depthImage.unproject(pt, DepthImage::TypeDisparity);
          if (!cropPoint(ptLocal, iBoundPlanes)) *outPtr = 0;
          else *outPtr = pt[2];
        }
      }
    }
    depthImage.setData(dispData, DepthImage::TypeDisparity);

    // wrap in view and return
    view->set(depthImage);
    return view;
  }
};

struct ViewWorker {
  typedef std::shared_ptr<ViewWorker> Ptr;

  BotWrapper::Ptr mBotWrapper;
  bool mActive;
  drc::map_request_t mRequest;
  std::shared_ptr<Collector> mCollector;
  std::shared_ptr<StereoHandler> mStereoHandlerHead;
  std::shared_ptr<StereoHandler> mStereoHandlerLeft;
  std::shared_ptr<StereoHandler> mStereoHandlerRight;
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
      if ((mRequest.view_id == drc::data_request_t::STEREO_MAP_HEAD) ||
          (mRequest.view_id == drc::data_request_t::STEREO_MAP_LHAND) ||
          (mRequest.view_id == drc::data_request_t::STEREO_MAP_RHAND)) {
        DepthImageView::Ptr view;
        switch (mRequest.view_id) {
        case drc::data_request_t::STEREO_MAP_HEAD:
          view = mStereoHandlerHead->getDepthImageView(spec.mClipPlanes);
          break;
        case drc::data_request_t::STEREO_MAP_LHAND:
          view = mStereoHandlerLeft->getDepthImageView(spec.mClipPlanes);
          break;
        case drc::data_request_t::STEREO_MAP_RHAND:
          view = mStereoHandlerRight->getDepthImageView(spec.mClipPlanes);
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
      }

      else if (localMap != NULL) {
        // do not send if there is not enough data
        // TODO: should make this cleaner; for now, 3 seconds
        int64_t timeMin = localMap->getPointData()->getTimeMin();
        int64_t timeMax = localMap->getPointData()->getTimeMax();
        if ((timeMax-timeMin) > 3000000) {

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
            DepthImage::AccumulationMethod accumMethod;
            // TODO: hack to differentiate between accumulation variants
            if (Utils::isOrthographic(projector.matrix())) {
              accumMethod = DepthImage::AccumulationMethodMean;
            }
            else {
              accumMethod = DepthImage::AccumulationMethodClosest;
            }
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
      std::this_thread::sleep_for
        (std::chrono::milliseconds(int(1000/mRequest.frequency)));
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
  std::shared_ptr<StereoHandler> mStereoHandlerLeft;
  std::shared_ptr<StereoHandler> mStereoHandlerRight;

  lcm::Subscription* mRequestSubscription;
  lcm::Subscription* mMapParamsSubscription;
  lcm::Subscription* mMapCommandSubscription;
  lcm::Subscription* mMapMacroSubscription;
  lcm::Subscription* mCatalogTriggerSubscription;

  float mCatalogPublishPeriod;

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
    mStereoHandlerLeft.reset(new StereoHandler(mBotWrapper, "CAMERA_LHAND"));
    mStereoHandlerRight.reset(new StereoHandler(mBotWrapper, "CAMERA_RHAND"));
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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    std::thread thread(worker);
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
      worker->mStereoHandlerLeft = mStereoHandlerLeft;
      worker->mStereoHandlerRight = mStereoHandlerRight;
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
  state.mCollector->getDataReceiver()->
    addChannel("SCAN", SensorDataReceiver::SensorTypePlanarLidar,
               "SCAN", "local");
  state.mCollector->bind("SCAN", 2);

  // set up remaining parameters
  LocalMap::Spec mapSpec;
  mapSpec.mId = 1;
  mapSpec.mPointBufferSize = 5000;
  mapSpec.mActive = true;
  mapSpec.mBoundMin = Eigen::Vector3f(-1,-1,-1)*1e10;
  mapSpec.mBoundMax = Eigen::Vector3f(1,1,1)*1e10;
  mapSpec.mResolution = defaultResolution;
  state.mCollector->getMapManager()->createMap(mapSpec);
  mapSpec.mId = 2;
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
  std::thread catalogThread(catalogSender);

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
  if (catalogThread.joinable()) catalogThread.join();

  return 0;
}
