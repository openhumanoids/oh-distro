#include <fstream>

#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>

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
    float angleMin = 90;
    float angleMax = 270;
    if (!mState->mCollector->getLatestSwath(angleMin*kDegToRad,
                                            angleMax*kDegToRad,
                                            timeMin, timeMax)) return;

    // if this time range overlaps the previous one, ignore
    if ((timeMin == mTimeMin) && (timeMax == mTimeMax)) return;
    mTimeMin = timeMin;
    mTimeMax = timeMax;
    std::cout << "got time range " << timeMin << " " << timeMax << std::endl;

    // create space-time bounds from desired time range
    // and a 6x6x6 data cube centered at (0,0,0)
    LocalMap::SpaceTimeBounds bounds;
    bounds.mTimeMin = timeMin;
    bounds.mTimeMax = timeMax;
    bounds.mPlanes = Utils::planesFromBox(Eigen::Vector3f(-10,-10,-10),
                                          Eigen::Vector3f(10,10,10));

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
    for (int i = 0; i < cloud->size(); ++i) {
      maps::PointCloud::PointType point = (*cloud)[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_switch_buffer(lcmgl);

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
      depthImageView->getDepthImage()->getData(DepthImage::TypeDisparity);
    std::ofstream ofs("/tmp/depths.txt");
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        ofs << depths[i*width + j] << " ";
      }
      ofs << std::endl;
    }
    ofs.close();
    std::cout << "Got depth image" << std::endl;

    // transmit compressed depth image over dummy lcm channel
    // note that the corresponding LcmTranslator::fromLcm() method
    // can be used by the receiver to decode the message when it arrives
    drc::map_image_t depthMsg;
    LcmTranslator::toLcm(*depthImageView, depthMsg);
    mState->mBotWrapper->getLcm()->publish("DUMMY_DEPTH", &depthMsg);
  }
};

int main() {
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
