#include "LidarAccumulator.hpp"

#include <fstream>
#include <chrono>
#include <thread>

#include <ConciseArgs>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/footstep_plan_progress_t.hpp>
#include <lcmtypes/drc/map_pointcloud_request_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>

#include <maps/PointCloudView.hpp>
#include <maps/LcmTranslator.hpp>

struct State {
  drc::LcmWrapper::Ptr mLcmWrapper;
  drc::BotWrapper::Ptr mBotWrapper;
  std::shared_ptr<maps::LidarAccumulator> mAccum;
  bool mRunContinuously;

  void setup() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mBotWrapper.reset(new drc::BotWrapper(mLcmWrapper->get()));
    mAccum.reset(new maps::LidarAccumulator());
    mAccum->setBotWrapper(mBotWrapper);
    auto lcm = mLcmWrapper->get();
    lcm->subscribe("FOOTSTEP_PLAN_PROGRESS", &State::onFootstepProgress, this);
    lcm->subscribe("MAP_POINTCLOUD_REQUEST", &State::onRequest, this);
  }

  void start() {
    if (mRunContinuously) mAccum->start();
    mLcmWrapper->startHandleThread(true);
  }

  void onFootstepProgress(const lcm::ReceiveBuffer* iBuf,
                          const std::string& iChannel,
                          const drc::footstep_plan_progress_t* iMessage) {
    if (mRunContinuously) return;
    if (mAccum->isRunning()) return;
    if ((iMessage->num_steps > 0) &&
        (iMessage->current_step == iMessage->num_steps)) {
      std::cout << "last footstep reached; starting accumulator" << std::endl;
      mAccum->start();
    }
  }

  void onRequest(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const drc::map_pointcloud_request_t* iMessage) {
    std::thread thread(&State::sendPointCloud, this, *iMessage);
    thread.detach();
  }

  void sendPointCloud(const drc::map_pointcloud_request_t& iMessage) {

    // integrate points
    std::vector<Eigen::Vector3f> points;
    mAccum->setRangeLimits(iMessage.min_range, iMessage.max_range);
    bool success =  mAccum->getPointCloud(iMessage.num_revolutions,
                                          iMessage.spindle_angle_start, points);

    // convert to pcl cloud
    maps::PointCloud::Ptr cloud(new maps::PointCloud());
    cloud->reserve(points.size());
    for (auto p : points) {
      maps::PointType pclPt;
      pclPt.getVector3fMap() = p;
      cloud->push_back(pclPt);
    }

    // set view
    maps::PointCloudView view;
    view.setResolution(iMessage.resolution);
    view.set(cloud);
    view.setId(iMessage.view_id);

    // get lcm message and publish
    drc::map_cloud_t msg;
    msg.utime = iMessage.utime;
    msg.map_id = 1;
    maps::LcmTranslator::toLcm(view, msg, 0.00001);
    mLcmWrapper->get()->publish("MAP_CLOUD", &msg);
    std::cout << "sent " << cloud->size() << " points" << std::endl;

    // stop collecting
    if (success && !mRunContinuously) mAccum->stop();
  }
};

int main(const int iArgc, const char** iArgv) {
  bool runContinuously = false;

  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(runContinuously, "c", "continuous", "run continuously");
  opt.parse();

  State state;
  state.mRunContinuously = runContinuously;
  state.setup();
  state.start();

  return 1;
}
