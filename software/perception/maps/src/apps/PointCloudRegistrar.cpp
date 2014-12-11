#include <iostream>
#include <unordered_set>
#include <thread>
#include <mutex>
#include <condition_variable>

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
  Eigen::Isometry3d mCurToRef;
  Eigen::Isometry3d mRefToOrig;
  std::vector<int> mRefInliers;
  bool mResetReference;
  std::thread mThread;
  std::mutex mCommandMutex;
  std::mutex mRunMutex;
  std::condition_variable mCondition;
  bool mIsRunning;
  bool mIsDataReady;

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
    mCurToRef = Eigen::Isometry3d::Identity();
    mRefToOrig = Eigen::Isometry3d::Identity();

    mLcm->subscribe("MAP_REGISTRATION_COMMAND", &State::onCommand, this);
    mResetReference = false;
    mIsRunning = false;
    mIsDataReady = false;
  }

  ~State() {
    stop();
    bot_lcmgl_destroy(mLcmGl);
  }

  void onCommand(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const drc::map_registration_command_t* iMessage) {
    if (iMessage->command == drc::map_registration_command_t::RESET) {
      std::unique_lock<std::mutex> lock(mCommandMutex);
      mResetReference = true;
      mCondition.notify_one();
    }
  }

  void reset() {
    std::unique_lock<std::mutex> lock(mCommandMutex);
    mRefCloud.reset();
    mCurToRef = Eigen::Isometry3d::Identity();
    mRefToOrig = Eigen::Isometry3d::Identity();
    mResetReference = false;
  }

  void operator()() {
    while (mIsRunning) {
      std::unique_lock<std::mutex> lock(mRunMutex);
      mCondition.wait_for(lock,std::chrono::seconds(1));
      if (mResetReference) reset();
      if (!mIsDataReady) continue;
      mIsDataReady = false;

      // create space-time bounds from desired time range and around head
      Eigen::Isometry3f headToLocal;
      mBotWrapper->getTransform("head","local",headToLocal);
      Eigen::Vector3f headPos = headToLocal.translation();
      maps::LocalMap::SpaceTimeBounds bounds;
      bounds.mTimeMin = mTimeMin;
      bounds.mTimeMax = mTimeMax;
      Eigen::Vector3f minPoint(-2,-2,-3), maxPoint(2,2,0.5);
      bounds.mPlanes =
        maps::Utils::planesFromBox(minPoint+headPos, maxPoint+headPos);

      // get point cloud corresponding to this time range
      maps::LocalMap::Ptr localMap =
        mCollector->getMapManager()->getMap(mActiveMapId);
      maps::PointCloudView::Ptr cloudView =
        localMap->getAsPointCloud(0.01, bounds);
      mCurCloud = cloudView->getAsPointCloud();

      if (mRefCloud == NULL) {
        mRefCloud = mCurCloud;
      }

      // align points
      Eigen::Isometry3f curToRef;
      double inlierFraction;
      if (registerToReference(mCurCloud, curToRef, inlierFraction)) {
        std::cout << "SUCCESS" << std::endl;
        mCurToRef = curToRef.cast<double>();

        // update reference if necessary
        if (inlierFraction < 0.3) {
          mRefCloud = mCurCloud;
          mRefToOrig = mRefToOrig*mCurToRef;
          mCurToRef = Eigen::Isometry3d::Identity();
          std::cout << "*** inlier fraction " << inlierFraction << 
            "; updating reference frame ***" << std::endl;
        }

        // publish pose update
        publishMessage(mRefToOrig*mCurToRef, mTimeMax);
      }
      else {
        std::cout << "FAILURE; resetting" << std::endl;
        reset();
      }
    }
  }

  void stop() {
    mIsRunning = false;
    mLcmWrapper->stopHandleThread();
    mCondition.notify_one();
    if (mThread.joinable()) mThread.join();
    mCollector->stop();
  }

  void start() {
    // create new submap
    maps::LocalMap::Spec mapSpec;
    mapSpec.mId = mActiveMapId;
    mapSpec.mPointBufferSize = 5000;
    mapSpec.mActive = true;
    mapSpec.mResolution = 0.01;
    mCollector->getMapManager()->createMap(mapSpec);

    // start running wrapper
    mCollector->getDataReceiver()->addChannel
      (mLaserChannel, maps::SensorDataReceiver::SensorTypePlanarLidar,
       mLaserChannel, "local");
    mCollector->addListener(*this);
    mCollector->start();

    // start running processing thread
    // (this avoids putting load on the data receiver thread)
    mIsRunning = true;
    mThread = std::thread(std::ref(*this));

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
    mIsDataReady = true;
    mCondition.notify_one();
  }

  void publishMessage(const Eigen::Isometry3d& iTransform,
                      const int64_t iTime) {
    bot_core::rigid_transform_t msg;
    msg.utime = iTime;
    for (int k = 0; k < 3; ++k) msg.trans[k] = iTransform(k,3);
    Eigen::Quaterniond q(iTransform.linear());
    msg.quat[0] = q.w();
    msg.quat[1] = q.x();
    msg.quat[2] = q.y();
    msg.quat[3] = q.z();
    mLcm->publish(mUpdateChannel, &msg);
  }

  void icpCallback(const maps::PointCloud& iSource,
                   const std::vector<int>& iIndicesSource,
                   const maps::PointCloud& iTarget,
                   const std::vector<int>& iIndicesTarget) {
    mRefInliers = iIndicesTarget;
  }

  bool registerToReference(const maps::PointCloud::Ptr& iCurCloud,
                           Eigen::Isometry3f& oCurToRef, double& oInlierFrac) {
    typedef maps::PointCloud::PointType PointType;
    if (mRefCloud == NULL) return false;

    // set up icp object
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputCloud(iCurCloud);
    icp.setInputTarget(mRefCloud);
    icp.setMaxCorrespondenceDistance(0.05);
    //icp.setUseReciprocalCorrespondences(true);
    boost::function
      <void(const maps::PointCloud&, const std::vector<int>&,
            const maps::PointCloud&, const std::vector<int>&)> functionObject;
    functionObject = boost::bind(&State::icpCallback, this, _1, _2, _3, _4);
    icp.registerVisualizationCallback(functionObject);
    pcl::PointCloud<PointType> finalCloud;

    // align clouds
    icp.align(finalCloud, mCurToRef.matrix().cast<float>());
    std::unordered_set<int> uniqueIndices;
    for (auto ind : mRefInliers) uniqueIndices.insert(ind);
    oCurToRef.matrix() = icp.getFinalTransformation();
    oInlierFrac = (double)uniqueIndices.size()/mRefCloud->size();

    std::cout << "score: " << icp.getFitnessScore() << std::endl;
    std::cout << oCurToRef.matrix() << std::endl;
    std::cout << "inlier fraction: " << oInlierFrac << std::endl;

    // debug
    /*
    bot_lcmgl_t* lcmgl = mLcmGl;
    bot_lcmgl_color4f(lcmgl, 0.5, 0, 0, 0.3);
    bot_lcmgl_point_size(lcmgl, 3);
    for (int i = 0; i < iCurCloud->size(); i += 1) {
      maps::PointCloud::PointType point = (*iCurCloud)[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_color4f(lcmgl, 0, 0.5, 0, 0.3);
    bot_lcmgl_point_size(lcmgl, 3);
    for (int i = 0; i < mRefCloud->size(); i += 1) {
      maps::PointCloud::PointType point = (*mRefCloud)[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_color4f(lcmgl, 1.0, 0.5, 0, 0.3);
    bot_lcmgl_point_size(lcmgl, 3);
    for (int i = 0; i < finalCloud.size(); i += 1) {
      maps::PointCloud::PointType point = finalCloud[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_switch_buffer(lcmgl);
*/

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
