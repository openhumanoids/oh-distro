#include <thread>
#include <condition_variable>
#include <mutex>
#include <sstream>

#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>

#include <bot_lcmgl_client/lcmgl.h>

#include <lcmtypes/drc/map_scans_t.hpp>
#include <lcmtypes/drc/affordance_collection_t.hpp>

#include <maps/ScanBundleView.hpp>
#include <maps/LcmTranslator.hpp>

#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>

#include "BlockFitter.hpp"

struct State {
  drc::BotWrapper::Ptr mBotWrapper;
  drc::LcmWrapper::Ptr mLcmWrapper;
  bool mRunContinuously;
  bool mDebug;

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
    mData.utime = 0;
    mLcmWrapper->get()->subscribe("MAP_SCANS", &State::onScans, this);
    mWorkerThread = std::thread(std::ref(*this));
    mLcmWrapper->startHandleThread(true);
  }

  void stop() {
    mLcmWrapper->stopHandleThread();
    if (mWorkerThread.joinable()) mWorkerThread.join();
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

      // remove points outside bbox
      planeseg::LabeledCloud::Ptr tempCloud(new planeseg::LabeledCloud());
      pcl::PassThrough<pcl::PointXYZL> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("y");
      pass.setFilterLimits (-3.0, 3.0);
      pass.filter(*tempCloud);
      std::swap(cloud, tempCloud);

      // process
      planeseg::BlockFitter fitter;
      fitter.setSensorPose(sensorPose.translation(),
                           sensorPose.rotation().col(2));
      fitter.setGroundBand(groundPose.translation()[2]-0.3,
                           groundPose.translation()[2]+0.5);
      fitter.setAreaThresholds(0.8, 1.2);
      fitter.setCloud(cloud);
      fitter.setDebug(mDebug);
      auto result = fitter.go();

      // construct json string
      std::string json;
      json += "{\n";
      json += "  \"command\": \"echo_response\",\n";
      json += "  \"descriptions\": {\n";
      std::string timeString = std::to_string(mBotWrapper->getCurrentTime());
      for (int i = 0; i < (int)result.mBlocks.size(); ++i) {
        const auto& block = result.mBlocks[i];
        std::string dimensionsString, positionString, quaternionString;
        {
          std::ostringstream oss;
          Eigen::Vector3f size = block.mSize;
          oss << size[0] << ", " << size[1] << ", " << size[2];
          dimensionsString = oss.str();
        }
        {
          std::ostringstream oss;
          Eigen::Vector3f p = block.mPose.translation();
          oss << p[0] << ", " << p[1] << ", " << p[2];
          positionString = oss.str();
        }
        {
          std::ostringstream oss;
          Eigen::Quaternionf q(block.mPose.rotation());
          oss << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z();
          quaternionString = oss.str();
        }
        std::string uuid = timeString + "_" + std::to_string(i+1);
        
        json += "    \"" + uuid + "\": {\n";
        json += "      \"classname\": \"BoxAffordanceItem\",\n";
        json += "      \"pose\": [[" + positionString + "], [" +
          quaternionString + "]],\n";
        json += "      \"uuid\": \"" + uuid + "\",\n";
        json += "      \"Dimensions\": [" + dimensionsString + "],\n";
        json += "      \"Name\": \"box" + std::to_string(i+1) + "\"\n";
        if (i == (int)result.mBlocks.size()-1) json += "    }\n";
        else json += "    },\n";
      }
      json += "  },\n";
      json += "  \"commandId\": \"" + timeString + "\",\n";
      json += "  \"collectionId\": \"block-fitter\"\n";
      json += "}\n";

      // publish result
      drc::affordance_collection_t msg;
      msg.utime = data.utime;
      msg.name = json;
      msg.naffs = 0;
      mLcmWrapper->get()->publish("AFFORDANCE_COLLECTION_COMMAND", &msg);
      std::cout << "Published affordance collection" << std::endl;

      // publish lcmgl
      if (mDebug) {
        bot_lcmgl_t* lcmgl;
        lcmgl = bot_lcmgl_init(mLcmWrapper->get()->getUnderlyingLCM(),
                              "block-fitter");
        for (const auto& block : result.mBlocks) {
          bot_lcmgl_color3f(lcmgl, 1, 0, 0);
          bot_lcmgl_line_width(lcmgl, 4);
          bot_lcmgl_begin(lcmgl, LCMGL_LINE_LOOP);
          for (const auto& pt : block.mHull) {
            bot_lcmgl_vertex3f(lcmgl, pt[0], pt[1], pt[2]);
          }
          bot_lcmgl_end(lcmgl);
        }
        bot_lcmgl_switch_buffer(lcmgl);
        bot_lcmgl_destroy(lcmgl);
      }

      if (!mRunContinuously) break;
    }
    mLcmWrapper->stopHandleThread();
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
  state.mRunContinuously = false;
  state.mDebug = false;

  state.start();
  state.stop();

  return 1;
}
