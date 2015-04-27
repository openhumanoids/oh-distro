#include <thread>
#include <condition_variable>
#include <mutex>

#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>

#include <lcmtypes/drc/map_scans_t.hpp>

#include <maps/ScanBundleView.hpp>
#include <maps/LcmTranslator.hpp>

#include <pcl/common/io.h>

#include "BlockFitter.hpp"

struct State {
  drc::BotWrapper::Ptr mBotWrapper;
  drc::LcmWrapper::Ptr mLcmWrapper;

  drc::map_scans_t mData;
  int64_t mLastDataTime;
  Eigen::Isometry3f mSensorPose;
  Eigen::Isometry3f mGroundPose;

  std::thread mWorkerThread;
  std::condition_variable mCondition;
  std::mutex mProcessMutex;
  std::mutex mDataMutex;

  void start() {
    mLastDataTime = 0;
    mLcmWrapper->get()->subscribe("MAP_SCANS", &State::onScans, this);
    mWorkerThread = std::thread(std::ref(*this));
    mLcmWrapper->startHandleThread(true);
  }

  void operator()() {
    while (true) {
      // wait for data
      std::unique_lock<std::mutex> lock(mProcessMutex);
      mCondition.wait_for(lock, std::chrono::milliseconds(100));

      // grab data
      drc::map_scans_t data;
      Eigen::Isometry3f sensorPose;
      Eigen::Isometry3f groundPose;
      {
        std::unique_lock<std::mutex> dataLock(mDataMutex);
        if (mData.utime <= mLastDataTime) continue;
        data = mData;
        sensorPose = mSensorPose;
        groundPose =mGroundPose;
        mLastDataTime = mData.utime;
      }

      // convert scans to point cloud
      maps::ScanBundleView view;
      maps::LcmTranslator::fromLcm(data, view);
      auto rawCloud = view.getAsPointCloud();
      planeseg::LabeledCloud::Ptr cloud(new planeseg::LabeledCloud());
      pcl::copyPointCloud(*rawCloud, *cloud);

      // process
      planeseg::BlockFitter fitter;
      fitter.setSensorPose(sensorPose.translation(),
                           sensorPose.rotation().col(2));
      fitter.setGroundBand(groundPose.translation()[2]-0.3,
                           groundPose.translation()[2]+0.5);
      fitter.setCloud(cloud);
      auto result = fitter.go();

      // publish result
      // TODO
    }
  }


  void onScans(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const drc::map_scans_t* iMessage) {
    std::unique_lock<std::mutex> lock(mDataMutex);
    mData = *iMessage;
    mBotWrapper->getTransform("head", "local", mSensorPose, iMessage->utime);
    mBotWrapper->getTransform("ground", "local", mGroundPose, iMessage->utime);
    mCondition.notify_one();
  }
};

int main(const int iArgc, const char** iArgv) {

  State state;
  state.mBotWrapper.reset(new drc::BotWrapper());
  state.mLcmWrapper.reset(new drc::LcmWrapper(state.mBotWrapper->getLcm()));
  
  state.start();

  return 1;
}
