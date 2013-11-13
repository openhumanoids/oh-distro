#include <iostream>
#include <unordered_map>
#include <thread>

#include <drc_utils/LcmWrapper.hpp>
#include <ConciseArgs>
#include <lcm/lcm-cpp.hpp>

#include <bot_lcmgl_client/lcmgl.h>

#include <maps/BotWrapper.hpp>
#include <maps/Collector.hpp>
#include <maps/Utils.hpp>
#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/PointCloudView.hpp>

#include <lcmtypes/bot_core/rigid_transform_t.hpp>
#include <lcmtypes/drc/map_registration_command_t.hpp>

#include <pcl/registration/icp.h>

struct State : public maps::Collector::DataListener {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<maps::BotWrapper> mBotWrapper;
  std::shared_ptr<maps::Collector> mCollector;
  std::string mLaserChannel;
  std::string mUpdateChannel;
  bot_lcmgl_t* mLcmGl;
  int64_t mActiveMapId;

  int64_t mTimeMin;
  int64_t mTimeMax;
  maps::PointCloud::Ptr mRefCloud;
  maps::PointCloud::Ptr mCurCloud;
  Eigen::Isometry3f mCurToRef;
  std::mutex mMutex;

  State() {
    // initialize some variables
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    mBotWrapper.reset(new maps::BotWrapper(mLcm));
    mCollector.reset(new maps::Collector());
    mCollector->setBotWrapper(mBotWrapper);
    mLaserChannel = "SCAN_FREE";
    mUpdateChannel = "MAP_LOCAL_CORRECTION";
    mActiveMapId = 1;
    mTimeMin = mTimeMax = 0;
    mLcmGl = bot_lcmgl_init(mBotWrapper->getLcm()->getUnderlyingLCM(),
                            "maps-registrar");
    mCurToRef = Eigen::Isometry3f::Identity();

    mLcm->subscribe("MAP_REGISTRATION_COMMAND", &State::onCommand, this);
  }

  ~State() {
    bot_lcmgl_destroy(mLcmGl);
  }

  void onCommand(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const drc::map_registration_command_t* iMessage) {
    if (iMessage->command == drc::map_registration_command_t::RESET_REFERENCE) {
      std::unique_lock<std::mutex> lock(mMutex);
      mRefCloud.reset();
      mCurToRef = Eigen::Isometry3f::Identity();
    }
  }

  void start() {
    // create new submap
    maps::LocalMap::Spec mapSpec;
    mapSpec.mId = mActiveMapId;
    mapSpec.mPointBufferSize = 5000;
    mapSpec.mActive = true;
    mapSpec.mBoundMin = Eigen::Vector3f(-1,-1,-1)*1e10;
    mapSpec.mBoundMax = Eigen::Vector3f(1,1,1)*1e10;
    mapSpec.mResolution = 0.01;
    mCollector->getMapManager()->createMap(mapSpec);

    // start running wrapper
    mCollector->getDataReceiver()->addChannel
      (mLaserChannel, maps::SensorDataReceiver::SensorTypePlanarLidar,
       mLaserChannel, "local");
    mCollector->addListener(*this);
    mCollector->start();

    // start lcm thread running
    mLcmWrapper->startHandleThread(true);
  }

  void notify(const maps::SensorDataReceiver::SensorData& iData) {
    const float kPi = 4*atan(1);
    const float kDegToRad = kPi/180;

    // find time range of swath and check if it overlaps previous one
    int64_t timeMin(0), timeMax(0);
    if (!mCollector->getLatestSwath(10*kDegToRad, 190*kDegToRad,
                                    timeMin, timeMax)) return;
    if ((timeMin == mTimeMin) && (timeMax == mTimeMax)) return;
    mTimeMin = timeMin;
    mTimeMax = timeMax;
    std::cout << "time range: " << timeMin << " " << timeMax << std::endl;

    // create space-time bounds from desired time range and around head
    Eigen::Isometry3f headToLocal;
    mBotWrapper->getTransform("head","local",headToLocal);
    Eigen::Vector3f headPos = headToLocal.translation();
    std::cout << "HEAD POS " << headPos.transpose() << std::endl;
    maps::LocalMap::SpaceTimeBounds bounds;
    bounds.mTimeMin = timeMin;
    bounds.mTimeMax = timeMax;
    Eigen::Vector3f minPoint(-2,-2,-3), maxPoint(2,2,0.5);
    bounds.mPlanes =
      maps::Utils::planesFromBox(minPoint+headPos, maxPoint+headPos);

    // get point cloud corresponding to this time range
    maps::LocalMap::Ptr localMap =
      mCollector->getMapManager()->getMap(mActiveMapId);
    maps::PointCloudView::Ptr cloudView =
      localMap->getAsPointCloud(0.01, bounds);
    mCurCloud = cloudView->getAsPointCloud();

    {
      // align clouds
      std::unique_lock<std::mutex> lock(mMutex);

      if (mRefCloud == NULL) {
        mRefCloud = mCurCloud;
      }

      Eigen::Isometry3f curToRef;
      if (registerToReference(mCurCloud, curToRef)) {
        std::cout << "SUCCESS" << std::endl;
        mCurToRef = curToRef;
        publishMessage(mCurToRef, timeMax);
      }
    }
  }

  void publishMessage(const Eigen::Isometry3f& iTransform,
                      const int64_t iTime) {
    bot_core::rigid_transform_t msg;
    msg.utime = iTime;
    for (int k = 0; k < 3; ++k) msg.trans[k] = iTransform(k,3);
    Eigen::Quaternionf q(iTransform.linear());
    msg.quat[0] = q.w();
    msg.quat[1] = q.x();
    msg.quat[2] = q.y();
    msg.quat[3] = q.z();
    mLcm->publish(mUpdateChannel, &msg);
  }

  bool registerToReference(const maps::PointCloud::Ptr& iCurCloud,
                           Eigen::Isometry3f& oCurToRef) {
    typedef maps::PointCloud::PointType PointType;
    if (mRefCloud == NULL) return false;
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputCloud(iCurCloud);
    icp.setInputTarget(mRefCloud);
    icp.setMaxCorrespondenceDistance(0.05);
    pcl::PointCloud<PointType> finalCloud;
    icp.align(finalCloud, mCurToRef.matrix());
    std::cout << "converged?:" << icp.hasConverged() << " score: " <<
      icp.getFitnessScore() << std::endl;
    std::cout << "matching points:" << finalCloud.size() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    oCurToRef.matrix() = icp.getFinalTransformation();

    // debug
    bot_lcmgl_t* lcmgl = mLcmGl;
    bot_lcmgl_color4f(lcmgl, 0.5, 0, 0, 0.3);
    bot_lcmgl_point_size(lcmgl, 3);
    for (int i = 0; i < iCurCloud->size(); ++i) {
      maps::PointCloud::PointType point = (*iCurCloud)[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_color4f(lcmgl, 0, 0.5, 0, 0.3);
    bot_lcmgl_point_size(lcmgl, 3);
    for (int i = 0; i < mRefCloud->size(); ++i) {
      maps::PointCloud::PointType point = (*mRefCloud)[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_color4f(lcmgl, 0, 0, 0.5,0.3);
    bot_lcmgl_point_size(lcmgl, 3);
    for (int i = 0; i < finalCloud.size(); ++i) {
      maps::PointCloud::PointType point = finalCloud[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_switch_buffer(lcmgl);

    return icp.hasConverged();
  }
};

int main(const int iArgc, const char** iArgv) {

  // instantiate state object
  State state;

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(state.mLaserChannel, "l", "laser_channel",
          "channel to use for incoming lidar data");
  opt.add(state.mUpdateChannel, "u", "update_channel",
          "channel on which to publish pose updates");
  opt.parse();

  state.start();

  return 0;
}
