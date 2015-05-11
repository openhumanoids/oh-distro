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
  bool mDoFilter;
  bool mRemoveGround;
  bool mDebug;
  Eigen::Vector3f mBlockSize;
  int mAlgorithm;  // TODO: use algo

  drc::map_scans_t mData;
  int64_t mLastDataTime;
  Eigen::Isometry3f mSensorPose;
  Eigen::Isometry3f mGroundPose;

  std::thread mWorkerThread;
  std::condition_variable mCondition;
  std::mutex mProcessMutex;
  std::mutex mDataMutex;

  State() {
    mRunContinuously = false;
    mDoFilter = true;
    mRemoveGround = true;
    mDebug = false;
    mAlgorithm = 0;  // TODO
    mBlockSize << 0, 0, 0;
  }

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
        groundPose = mGroundPose;
        mLastDataTime = mData.utime;
      }

      // convert scans to point cloud
      // TODO: could do this scan by scan and save away high deltas
      maps::ScanBundleView view;
      maps::LcmTranslator::fromLcm(data, view);
      auto rawCloud = view.getAsPointCloud();
      planeseg::LabeledCloud::Ptr cloud(new planeseg::LabeledCloud());
      pcl::copyPointCloud(*rawCloud, *cloud);

      // remove points outside max radius
      const float kValidRadius = 5;  // meters; TODO: could make this a param
      const float kValidRadius2 = kValidRadius*kValidRadius;
      planeseg::LabeledCloud::Ptr tempCloud(new planeseg::LabeledCloud());
      for (int i = 0; i < (int)cloud->size(); ++i) {
        Eigen::Vector3f p = cloud->points[i].getVector3fMap();
        float dist2 = (p-sensorPose.translation()).squaredNorm();
        if (dist2 > kValidRadius2) continue;
        tempCloud->push_back(cloud->points[i]);
      }
      std::swap(cloud, tempCloud);

      // process
      planeseg::BlockFitter fitter;
      fitter.setSensorPose(sensorPose.translation(),
                           sensorPose.rotation().col(2));
      fitter.setGroundBand(groundPose.translation()[2]-1.0,
                           groundPose.translation()[2]+0.5);
      if (mDoFilter) fitter.setAreaThresholds(0.8, 1.2);
      else fitter.setAreaThresholds(0, 1000);
      if (mBlockSize.norm() > 1e-5) fitter.setBlockDimensions(mBlockSize);
      fitter.setRemoveGround(mRemoveGround);
      fitter.setDebug(mDebug);
      fitter.setCloud(cloud);
      auto result = fitter.go();
      if (!result.mSuccess) {
        std::cout << "error: could not detect blocks" << std::endl;
        continue;
      }

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
        json += "      \"Name\": \"cinderblock " + std::to_string(i) + "\"\n";
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

        bot_lcmgl_color3f(lcmgl, 0, 1, 0);
        bot_lcmgl_begin(lcmgl, LCMGL_LINE_LOOP);
        for (const auto& pt : result.mGroundPolygon) {
          bot_lcmgl_vertex3f(lcmgl, pt[0], pt[1], pt[2]);
        }
        bot_lcmgl_end(lcmgl);

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
    int64_t scanTime = iMessage->utime;
    int64_t headPoseTime = mBotWrapper->getLatestTime("head", "local");
    int64_t groundPoseTime = mBotWrapper->getLatestTime("ground", "local");
    if ((std::abs(headPoseTime-scanTime) > 1e6) ||
        (std::abs(groundPoseTime-scanTime) > 1e6)) {
      std::cout << "warning: got scans but no valid pose found" << std::endl;
      return;
    }
    mBotWrapper->getTransform("head", "local", mSensorPose, iMessage->utime);
    mBotWrapper->getTransform("ground", "local", mGroundPose, iMessage->utime);
    mCondition.notify_one();
  }
};

int main(const int iArgc, const char** iArgv) {

  std::string sizeString("");
  State state;

  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(state.mRunContinuously, "c", "continuous", "run continuously");
  opt.add(state.mDoFilter, "f", "filter", "filter blocks based on size");
  opt.add(sizeString, "s", "blocksize", "prior size for blocks \"x y z\"");
  opt.add(state.mRemoveGround, "g", "remove-ground",
          "whether to remove ground before processing");
  opt.add(state.mAlgorithm, "a", "algorithm",
          "0=min_area, 1=closest_size, 2=closest_hull");
  opt.add(state.mDebug, "d", "debug", "debug flag");
  opt.parse();

  if (sizeString.length() > 0) {
    std::istringstream iss(sizeString);
    float x, y, z;
    if (iss >> x) {
      if (iss >> y) {
        if (iss >> z) {
          state.mBlockSize << x,y,z;
          std::cout << "using block size " << state.mBlockSize.transpose() <<
            std::endl;
        }
      }
    }
  }

  state.mBotWrapper.reset(new drc::BotWrapper());
  state.mLcmWrapper.reset(new drc::LcmWrapper(state.mBotWrapper->getLcm()));

  state.start();
  state.stop();

  return 1;
}
