#include <fstream>
#include <chrono>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/data_request_t.hpp>

#include <maps/SensorDataReceiver.hpp>
#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/Collector.hpp>
#include <maps/PointCloudView.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/LcmTranslator.hpp>
#include <maps/Utils.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/DepthImage.hpp>
#include <maps/ObjectPool.hpp>

#include <maps/ViewClient.hpp>
#include <bot_param/param_client.h>

// TODO TEMP
#include <bot_core/timestamp.h>
#include <drc_utils/Clock.hpp>
#include <drc_utils/LidarUtils.hpp>

using namespace maps;
using namespace std;

class State {
public:
  BotWrapper::Ptr mBotWrapper;
  std::shared_ptr<Collector> mCollector;
  int mActiveMapId;
  bot_lcmgl_t* mLcmGl;

  State() {
    mBotWrapper.reset(new BotWrapper());
    mCollector.reset(new Collector());
    mCollector->setBotWrapper(mBotWrapper);
    mActiveMapId = 0;
    mLcmGl = bot_lcmgl_init(mBotWrapper->getLcm()->getUnderlyingLCM(),
                            "test-collector");
  }

  ~State() {
    bot_lcmgl_destroy(mLcmGl);
  }
};

class DataProducer : public Collector::DataListener {
protected:
  State* mState;
  int64_t mTimeMin;
  int64_t mTimeMax;
public:
  DataProducer(State* iState) : mState(iState), mTimeMin(0), mTimeMax(0) {}

  void notify(const SensorDataReceiver::SensorData& iData) {
    const float kPi = 4*atan(1);
    const float kDegToRad = kPi/180;

    // get submap we created earlier
    LocalMap::Ptr localMap =
      mState->mCollector->getMapManager()->getMap(mState->mActiveMapId);

    // find time range of desired swath (from 45 to 135 degrees)
    // note that 0 and 180 degrees are equivalent
    int64_t timeMin(0), timeMax(0);
    if (!mState->mCollector->getLatestSwath(0*kDegToRad, 179.99*kDegToRad,
                                            timeMin, timeMax)) return;

    // if this time range overlaps the previous one, ignore
    if ((timeMin <= mTimeMax) && (timeMax >= mTimeMin)) return;
    mTimeMin = timeMin;
    mTimeMax = timeMax;
    std::cout << "got time range " << timeMin << " " << timeMax << std::endl;

    // create space-time bounds from desired time range
    // and a 6x6x6 data cube centered at (0,0,0)
    LocalMap::SpaceTimeBounds bounds;
    bounds.mTimeMin = timeMin;
    bounds.mTimeMax = timeMax;
    bounds.mPlanes = Utils::planesFromBox(Eigen::Vector3f(-5,-5,-5),
                                          Eigen::Vector3f(5,5,5));

    // get point cloud corresponding to this time range
    PointCloudView::Ptr cloudView = localMap->getAsPointCloud(0, bounds);
    drc::map_cloud_t cloudMessage;

    // publish compressed point cloud view over dummy lcm channel
    LcmTranslator::toLcm(*cloudView, cloudMessage);
    mState->mBotWrapper->getLcm()->publish("DUMMY_CLOUD", &cloudMessage);

    // publish raw cloud as lcmgl
    maps::PointCloud::Ptr cloud = cloudView->getPointCloud();    
    bot_lcmgl_t* lcmgl = mState->mLcmGl;
    bot_lcmgl_color3f(lcmgl, 0, 1, 0);
    bot_lcmgl_point_size(lcmgl, 3);
    std::ofstream ofs("/tmp/points.txt");
    for (int i = 0; i < cloud->size(); ++i) {
      maps::PointCloud::PointType point = (*cloud)[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
      ofs << point.getVector3fMap().transpose() << std::endl;
    }
    bot_lcmgl_switch_buffer(lcmgl);
    ofs.close();

    // set up sample camera pose for depth image
    Eigen::Vector3f trans(0,0,0);   // camera position wrt world
    Eigen::Matrix3f rot;            // camera orientation wrt world
    rot.col(0) = -Eigen::Vector3f::UnitY();
    rot.col(1) = -Eigen::Vector3f::UnitZ();
    rot.col(2) = Eigen::Vector3f::UnitX();
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = rot;
    pose.translation() = trans;

    // set up sample camera projection parameters for depth image
    int width(200), height(200);
    Eigen::Matrix3f calib = Eigen::Matrix3f::Identity();
    calib(0,0) = calib(1,1) = 50;  // focal length of 50 pixels
    calib(0,2) = width/2.0;        // cop at center of image
    calib(1,2) = height/2.0;

    // create depth image
    Eigen::Projective3f projector;
    Utils::composeViewMatrix(projector, calib, pose, false);
    DepthImageView::Ptr depthImageView =
      localMap->getAsDepthImage(width, height, projector, bounds);

    drc::map_image_t depthMsg;
    LcmTranslator::toLcm(*depthImageView, depthMsg);
    LcmTranslator::fromLcm(depthMsg, *depthImageView);

    std::cout << "PROJECTOR\n" << depthImageView->getTransform().matrix() << std::endl;

    // get point cloud from depth image view (in ref coords)
    cloud = depthImageView->getAsPointCloud();
    bot_lcmgl_color3f(lcmgl, 0, 0.5, 0);
    bot_lcmgl_point_size(lcmgl, 3);
    for (int i = 0; i < cloud->size(); ++i) {
      maps::PointCloud::PointType point = (*cloud)[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_switch_buffer(lcmgl);

    // get raw depth image pixel values and store to file
    std::vector<float> depths =
      depthImageView->getDepthImage()->getData(DepthImage::TypeDepth);
    std::vector<float> disparities =
      depthImageView->getDepthImage()->getData(DepthImage::TypeDisparity);
    std::vector<float> ranges =
      depthImageView->getDepthImage()->getData(DepthImage::TypeRange);
    ofs.open("/tmp/depths.txt");
    ofstream ofs2("/tmp/disparities.txt");
    ofstream ofs3("/tmp/ranges.txt");
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        ofs << depths[i*width + j] << " ";
        ofs2 << disparities[i*width + j] << " ";
        ofs3 << ranges[i*width + j] << " ";
      }
      ofs << std::endl;
      ofs2 << std::endl;
      ofs3 << std::endl;
    }
    ofs.close();
    std::cout << "Got depth image" << std::endl;

    // transmit compressed depth image over dummy lcm channel
    // note that the corresponding LcmTranslator::fromLcm() method
    // can be used by the receiver to decode the message when it arrives
    LcmTranslator::toLcm(*depthImageView, depthMsg);
    mState->mBotWrapper->getLcm()->publish("DUMMY_DEPTH", &depthMsg);
  }
};




struct Helper {
  std::shared_ptr<lcm::LCM> mLcm;
  maps::ViewClient* mViewClient;
  std::thread mThread;
  bool mIsRunning;

  void operator()() {
    while (mIsRunning) {
      int fn = mLcm->getFileno();
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(fn, &fds);
      struct timeval timeout = { 0, 200*1000 };
      int status = select(fn+1, &fds, NULL, NULL, &timeout);
      if (status == 0) {
      }
      else if (status < 0) {
        break;
      }
      else if (FD_ISSET(fn, &fds)) {
        if (0 != mLcm->handle()) {
          break;
        }
      }
    }
  }
};


int main() {
  const double kPi = 4*atan(1);
  const double fov = 270;
  const int numPoints = 1000;
  std::vector<Eigen::Vector3f> pts;
  Eigen::Isometry3d pose0 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() << 0,0,0;
  pose1.linear() = Eigen::AngleAxisd(-fov/2*kPi/180,
                                     Eigen::Vector3d(0,0,1)).matrix();
  std::vector<float> ranges(numPoints);
  for (int i = 0; i < ranges.size(); ++i) {
    ranges[i] = 3;
  }
  drc::LidarUtils::interpolateScan(ranges, -fov/2*kPi/180,
                                   fov/numPoints*kPi/180, pose0, pose1, pts);
  for (auto pt : pts) {
    std::cout << pt.transpose() << std::endl;
  }
  return -1;

  maps::ObjectPool<DepthImageView,5> pool;
  std::vector<std::shared_ptr<DepthImageView> > pointers;
  for (int i = 0; i < pool.getCapacity()+3; ++i) {
    auto testPtr = pool.get();
    if (testPtr != NULL) {
      std::cout << "SUCCESS" << std::endl;
      std::cout << "NUM FREE " << pool.getNumFree() << std::endl;
      pointers.push_back(testPtr);
    }
    else {
      std::cout << "FAILURE" << std::endl;
    }
  }

  return 0;


  std::shared_ptr<Helper> helper(new Helper());
  helper->mViewClient = new maps::ViewClient();
  helper->mLcm.reset(new lcm::LCM());
  if ((NULL == helper->mLcm) || !helper->mLcm->good()) {
    std::cout << "cannot create lcm" << std::endl;
    return -1;
  }

  BotWrapper::Ptr botWrapper(new BotWrapper(helper->mLcm, NULL, NULL));
  if (NULL == botWrapper->getBotParam()) {
    std::cout << "cannot create boparam" << std::endl;
    return -1;
  }

  helper->mIsRunning = true;
  helper->mThread = std::thread(std::ref(*helper));


  helper->mViewClient->setBotWrapper(botWrapper);
  helper->mViewClient->start();

  sleep(3);

  DepthImageView::Ptr view = std::dynamic_pointer_cast<DepthImageView>
    (helper->mViewClient->getView(drc::data_request_t::HEIGHT_MAP_SCENE));
  view->setNormalRadius(2);
  view->setNormalMethod(DepthImageView::NormalMethodLeastSquares);
  
  State s;
  bot_lcmgl_t* lcmgl = s.mLcmGl;
  std::vector<Eigen::Vector3f> points, normals;
  auto startTime = std::chrono::high_resolution_clock::now();
  for (float y = -10; y <= 10; y += 0.1) {
    for (float x = -10; x <= 10; x += 0.1) {
      Eigen::Vector3f queryPt(x,y,0);
      Eigen::Vector3f pt, normal;
      if (view->getClosest(queryPt, pt, normal)) {
        points.push_back(pt);
        normals.push_back(normal);
        std::cout << normal.transpose() << std::endl;
      }
    }
  }
  auto endTime = std::chrono::high_resolution_clock::now();
  auto timeDiff =
    std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "Time elapsed " << timeDiff.count()/1e3 << std::endl;

  std::cout << "POINTS " << points.size() << std::endl;

  bot_lcmgl_color3f(lcmgl, 0, 1, 0);
  bot_lcmgl_point_size(lcmgl, 10);
  bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
  for (int i = 0; i < points.size(); ++i) {
    bot_lcmgl_vertex3f(lcmgl, points[i][0], points[i][1], points[i][2]);
  }
  bot_lcmgl_end(lcmgl);

  bot_lcmgl_color3f(lcmgl, 0, 0, 1);
  bot_lcmgl_line_width(lcmgl, 2);
  bot_lcmgl_begin(lcmgl, LCMGL_LINES);
  for (int i = 0; i < normals.size(); ++i) {
    bot_lcmgl_vertex3f(lcmgl, points[i][0], points[i][1], points[i][2]);
    Eigen::Vector3f pt = points[i] + 0.1*normals[i];
    bot_lcmgl_vertex3f(lcmgl, pt[0], pt[1], pt[2]);
  }
  bot_lcmgl_end(lcmgl);

  bot_lcmgl_switch_buffer(lcmgl);

  //  sleep(5);

  helper->mViewClient->stop();
  delete helper->mViewClient;

  helper->mIsRunning = false;
  if (helper->mThread.joinable()) helper->mThread.join();
  helper.reset();

  return 0;

  // create state object instance
  State state;

  // create new submap
  LocalMap::Spec mapSpec;
  mapSpec.mId = 1;
  mapSpec.mPointBufferSize = 5000;
  mapSpec.mActive = true;
  mapSpec.mResolution = 0.01;
  state.mActiveMapId = state.mCollector->getMapManager()->createMap(mapSpec);

  // start running wrapper
  std::string laserChannel("SCAN");
  state.mCollector->getDataReceiver()->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");
  state.mCollector->start();

  DataProducer producer(&state);;
  state.mCollector->addListener(producer);

  // main lcm loop
  while (0 == state.mBotWrapper->getLcm()->handle());
}
