#include "SensorDataReceiver.hpp"

#include <unordered_map>
#include <list>
#include <thread>
#include <mutex>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core/planar_lidar_t.hpp>

#include "Types.hpp"
#include "LidarScan.hpp"
#include "ThreadSafeQueue.hpp"
#include "BotWrapper.hpp"

#include <pcl/io/io.h>
#include <bot_param/param_util.h>

using namespace maps;

struct SensorDataReceiver::Helper {
  struct SubscriptionInfo {
    std::string mSensorChannel;
    SensorType mSensorType;
    std::string mTransformFrom;
    std::string mTransformTo;
    float mRangeMin;
    float mRangeMax;
    float mFrequency;
    lcm::Subscription* mSubscription;
    typedef std::shared_ptr<SubscriptionInfo> Ptr;
  };

  struct PoseUpdater {
    Helper* mHelper;
    std::thread mThread;
    std::mutex mMutex;
    std::list<SensorData> mPendingData;
    void operator()() {
      SensorData data;
      while (mHelper->mIsRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        {
          std::lock_guard<std::mutex> lock(mMutex);
          if (mPendingData.size() == 0) continue;
          std::list<SensorData>::iterator iter = mPendingData.begin();
          while (iter != mPendingData.end()) {
            SubscriptionMap::const_iterator item =
              mHelper->mSubscriptions.find(iter->mChannel);
            if (item != mHelper->mSubscriptions.end()) {
              std::shared_ptr<SubscriptionInfo> info = item->second;
              int64_t latestTime = mHelper->mBotWrapper->getLatestTime
                (info->mTransformFrom, info->mTransformTo);
              int64_t dataTime = iter->mPointSet->mTimestamp;
              if (dataTime <= latestTime) {
                maps::PointCloud::Ptr& cloud = iter->mPointSet->mCloud;
                if (mHelper->getPose(info, dataTime, cloud->sensor_origin_,
                                     cloud->sensor_orientation_)) {
                  Eigen::Isometry3f poseEnd = Eigen::Isometry3f::Identity();
                  poseEnd.translation() = cloud->sensor_origin_.head<3>();
                  poseEnd.linear() = cloud->sensor_orientation_.matrix();

                  Eigen::Isometry3f poseStart;
                  float angleRange = (iter->mScan->getNumRanges()-1) *
                    iter->mScan->getThetaStep();
                  float scanTime = angleRange/(2*M_PI*info->mFrequency);
                  int64_t startTime = dataTime - int64_t(scanTime*1e-6f + 0.5);
                  mHelper->mBotWrapper->getTransform(info->mTransformFrom,
                                                     info->mTransformTo,
                                                     poseStart, startTime);
                                                
                  iter->mScan->setPoses(poseStart, poseEnd);
                  data = *iter;
                  mPendingData.erase(iter++);
                  data.mSensorType = info->mSensorType;
                  mHelper->mDataBuffer.push(data);
                  continue;
                }
              }
            }
            ++iter;
          }
        }
      }
    }
  };

  typedef std::unordered_map<std::string, std::shared_ptr<SubscriptionInfo> >
  SubscriptionMap;

  std::shared_ptr<BotWrapper> mBotWrapper;
  SubscriptionMap mSubscriptions;
  bool mIsRunning;
  ThreadSafeQueue<SensorData> mDataBuffer;
  ThreadSafeQueue<SensorData> mImmediateBuffer;
  std::mutex mBufferMutex;
  std::mutex mSubscriptionsMutex; // TODO: can probably get rid of this
  PoseUpdater mPoseUpdater;

  bool getPose(const std::shared_ptr<SubscriptionInfo>& iInfo,
               const int64_t iTimestamp,
               Eigen::Vector4f& oPosition, Eigen::Quaternionf& oOrientation) {
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    if (!mBotWrapper->getTransform(iInfo->mTransformFrom, iInfo->mTransformTo,
                                   rotation, translation, iTimestamp)) {
      std::cerr << "SensorDataReceiver: cannot get transform from " <<
        iInfo->mTransformFrom << " to " << iInfo->mTransformTo << std::endl;
      return false;
    }
    oPosition.head<3>() = translation;
    oPosition[3] = 1;
    oOrientation = rotation;
    return true;
  }

  void onLidar(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const bot_core::planar_lidar_t* iMessage) {
    if (!mIsRunning) return;

    std::shared_ptr<SubscriptionInfo> info;
    {
      std::lock_guard<std::mutex> lock(mSubscriptionsMutex);
      SubscriptionMap::const_iterator item = mSubscriptions.find(iChannel);
      if (item == mSubscriptions.end()) return;
      info = item->second;
    }

    maps::PointCloud::Ptr cloud(new maps::PointCloud());
    LidarScan::Ptr scan(new LidarScan());
    scan->setTimestamp(iMessage->utime);
    scan->setAngles(iMessage->rad0, iMessage->radstep);
    std::vector<float> ranges(iMessage->nranges);
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.reserve(iMessage->nranges);
    for (int i = 0; i < iMessage->nranges; ++i) {
      double theta = iMessage->rad0 + i*iMessage->radstep;
      double range = iMessage->ranges[i];
      ranges[i] = range;
      if ((range < info->mRangeMin) || (range > info->mRangeMax)) continue;
      PointType pt;
      pt.x = cos(theta)*range;
      pt.y = sin(theta)*range;
      pt.z = 0;
      cloud->points.push_back(pt);
    }
    cloud->width = cloud->points.size();
    scan->setRanges(ranges);
    scan->setIntensities(iMessage->intensities);

    SensorData data;
    data.mPointSet.reset(new PointSet());
    data.mPointSet->mTimestamp = iMessage->utime;
    data.mPointSet->mCloud = cloud;
    data.mScan = scan;
    data.mChannel = iChannel;
    mImmediateBuffer.push(data);
    {
      std::lock_guard<std::mutex> lock(mPoseUpdater.mMutex);
      mPoseUpdater.mPendingData.push_back(data);
    }
  }
};


SensorDataReceiver::
SensorDataReceiver() {
  mHelper.reset(new Helper());
  mHelper->mIsRunning = false;
  mHelper->mPoseUpdater.mHelper = mHelper.get();
  setMaxBufferSize(100);
}

SensorDataReceiver::
~SensorDataReceiver() {
  clearChannels();
}

void SensorDataReceiver::
setBotWrapper(const std::shared_ptr<BotWrapper>& iWrapper) {
  mHelper->mBotWrapper = iWrapper;
  clearChannels();
}

bool SensorDataReceiver::
addChannel(const std::string& iSensorChannel,
           const SensorType iSensorType,
           const std::string& iTransformFrom,
           const std::string& iTransformTo) {
  Helper::SubscriptionInfo::Ptr info(new Helper::SubscriptionInfo());
  info->mSensorChannel = iSensorChannel;
  info->mSensorType = iSensorType;
  info->mTransformFrom = iTransformFrom;
  info->mTransformTo = iTransformTo;
  info->mRangeMin = 0;
  info->mRangeMax = 1e10;
  info->mFrequency = 40;

  // get min and max values from config
  BotParam* botParam = mHelper->mBotWrapper->getBotParam();
  char* lidarName =
    bot_param_get_planar_lidar_name_from_lcm_channel(botParam,
                                                     iSensorChannel.c_str());
  char prefix[1024];
  bot_param_get_planar_lidar_prefix(NULL, lidarName, prefix, sizeof(prefix));
  std::string key = std::string(prefix) + ".max_range";
  double val;
  if (0 == bot_param_get_double(botParam, key.c_str(), &val)) {
    info->mRangeMax = val;
  }
  key = std::string(prefix) + ".min_range";
  if (0 == bot_param_get_double(botParam, key.c_str(), &val)) {
    info->mRangeMin = val;
  }
  key = std::string(prefix) + ".frequency";
  if (0 == bot_param_get_double(botParam, key.c_str(), &val)) {
    info->mFrequency = val;
  }
  free(lidarName);

  // subscribe to appropriate channel
  lcm::Subscription* sub = NULL;
  switch(iSensorType) {
  case SensorTypePlanarLidar:
    sub = mHelper->mBotWrapper->getLcm()->subscribe
      (iSensorChannel, &SensorDataReceiver::Helper::onLidar, mHelper.get());
    break;
  default:
    return false;
  }
  info->mSubscription = sub;
  {
    std::lock_guard<std::mutex> lock(mHelper->mSubscriptionsMutex);
    mHelper->mSubscriptions[info->mSensorChannel] = info;
  }

  return true;
}

void SensorDataReceiver::
clearChannels() {
  std::lock_guard<std::mutex> lock(mHelper->mSubscriptionsMutex);
  Helper::SubscriptionMap::const_iterator iter;
  for (iter = mHelper->mSubscriptions.begin();
       iter != mHelper->mSubscriptions.end(); ++iter) {
    mHelper->mBotWrapper->getLcm()->unsubscribe(iter->second->mSubscription);
  }
  mHelper->mSubscriptions.clear();
}

bool SensorDataReceiver::
removeChannel(const std::string& iSensorChannel) {
  std::lock_guard<std::mutex> lock(mHelper->mSubscriptionsMutex);
  Helper::SubscriptionMap::iterator item =
    mHelper->mSubscriptions.find(iSensorChannel);
  if (item == mHelper->mSubscriptions.end()) {
    return false;
  }
  mHelper->mBotWrapper->getLcm()->unsubscribe(item->second->mSubscription);
  mHelper->mSubscriptions.erase(item);
  return true;
}

void SensorDataReceiver::
setMaxBufferSize(const int iSize) {
  mHelper->mDataBuffer.setMaxSize(iSize);
  mHelper->mImmediateBuffer.setMaxSize(iSize);
}

bool SensorDataReceiver::
pop(SensorData& oData, const bool iNeedPose) {
  if (iNeedPose) return mHelper->mDataBuffer.pop(oData);
  else return mHelper->mImmediateBuffer.pop(oData);
}

bool SensorDataReceiver::
waitForData(SensorData& oData, const bool iNeedPose) {
  if (iNeedPose) return mHelper->mDataBuffer.waitForData(oData);
  return mHelper->mImmediateBuffer.waitForData(oData);
}

void SensorDataReceiver::
unblock() {
  mHelper->mDataBuffer.unblock();
  mHelper->mImmediateBuffer.unblock();
}

bool SensorDataReceiver::
start() {
  if (mHelper->mIsRunning) return false;
  mHelper->mIsRunning = true;
  mHelper->mPoseUpdater.mThread =
    std::thread(std::ref(mHelper->mPoseUpdater));
  return true;
}

bool SensorDataReceiver::
stop() {
  if (!mHelper->mIsRunning) return false;
  mHelper->mIsRunning = false;
  if (mHelper->mPoseUpdater.mThread.joinable()) {
    mHelper->mPoseUpdater.mThread.join();
  }
  return true;
}
