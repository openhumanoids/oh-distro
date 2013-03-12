#include "SensorDataReceiver.hpp"

#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/drc/pointcloud2_t.hpp>

#include "Types.hpp"
#include "ThreadSafeQueue.hpp"
#include "BotFramesWrapper.hpp"

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
    lcm::Subscription* mSubscription;
    typedef boost::shared_ptr<SubscriptionInfo> Ptr;
  };

  struct PoseUpdater {
    Helper* mHelper;
    boost::thread mThread;
    boost::mutex mMutex;
    std::list<SensorData> mPendingData;
    void operator()() {
      SensorData data;
      while (mHelper->mIsRunning) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        {
          boost::mutex::scoped_lock lock(mMutex);
          if (mPendingData.size() == 0) continue;
          std::list<SensorData>::iterator iter = mPendingData.begin();
          while (iter != mPendingData.end()) {
            SubscriptionMap::const_iterator item =
              mHelper->mSubscriptions.find(iter->mChannel);
            if (item != mHelper->mSubscriptions.end()) {
              boost::shared_ptr<SubscriptionInfo> info = item->second;
              int64_t latestTime = mHelper->mBotFrames->getLatestTimestamp
                (info->mTransformFrom, info->mTransformTo);
              int64_t dataTime = iter->mPointSet->mTimestamp;
              if (dataTime <= latestTime) {
                maps::PointCloud::Ptr& cloud = iter->mPointSet->mCloud;
                if (mHelper->getPose(info, dataTime, cloud->sensor_origin_,
                                     cloud->sensor_orientation_)) {
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

  typedef std::unordered_map<std::string, boost::shared_ptr<SubscriptionInfo> >
  SubscriptionMap;

  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<BotFramesWrapper> mBotFrames;
  BotParam* mBotParam;
  SubscriptionMap mSubscriptions;
  bool mIsRunning;
  ThreadSafeQueue<SensorData> mDataBuffer;
  ThreadSafeQueue<SensorData> mImmediateBuffer;
  boost::mutex mBufferMutex;
  boost::mutex mSubscriptionsMutex; // TODO: can probably get rid of this
  PoseUpdater mPoseUpdater;

  bool getPose(const boost::shared_ptr<SubscriptionInfo>& iInfo,
               const int64_t iTimestamp,
               Eigen::Vector4f& oPosition, Eigen::Quaternionf& oOrientation) {
    Eigen::Vector3f trans;
    Eigen::Quaternionf rot;
    if (!mBotFrames->getTransform(iInfo->mTransformFrom, iInfo->mTransformTo,
                                  iTimestamp, trans, rot)) {
      std::cerr << "SensorDataReceiver: cannot get transform from " <<
        iInfo->mTransformFrom << " to " << iInfo->mTransformTo << std::endl;
      return false;
    }

    oPosition.head<3>() = trans;
    oPosition[3] = 1;
    oOrientation = rot;
    return true;
  }

  void onCloud(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const drc::pointcloud2_t* iMessage) {
    if (!mIsRunning) return;

    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pointCloud.width = iMessage->width;
    pointCloud.height = iMessage->height;
    pointCloud.resize(iMessage->width * iMessage->height);
    pointCloud.is_dense = false;
    uint8_t* cloudData = (uint8_t*)(&pointCloud[0]);
    const uint8_t* messageData = (uint8_t*)(&iMessage->data[0]);
    memcpy(cloudData, messageData, iMessage->data_nbytes);
    maps::PointCloud::Ptr newCloud(new maps::PointCloud());
    pcl::copyPointCloud(pointCloud, *newCloud);

    SensorData data;
    data.mPointSet.reset(new PointSet());
    data.mPointSet->mTimestamp = iMessage->utime;
    data.mPointSet->mMinRange = 1e10;
    data.mPointSet->mMaxRange = 1e10;
    data.mPointSet->mCloud = newCloud;
    data.mChannel = iChannel;
    mImmediateBuffer.push(data);
    {
      boost::mutex::scoped_lock lock(mPoseUpdater.mMutex);
      mPoseUpdater.mPendingData.push_back(data);
    }
  }

  void onLidar(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const bot_core::planar_lidar_t* iMessage) {
    if (!mIsRunning) return;

    boost::shared_ptr<SubscriptionInfo> info;
    {
      boost::mutex::scoped_lock lock(mSubscriptionsMutex);
      SubscriptionMap::const_iterator item = mSubscriptions.find(iChannel);
      if (item == mSubscriptions.end()) return;
      info = item->second;
    }

    maps::PointCloud::Ptr cloud(new maps::PointCloud());
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.reserve(iMessage->nranges);
    for (int i = 0; i < iMessage->nranges; ++i) {
      double theta = iMessage->rad0 + i*iMessage->radstep;
      double range = iMessage->ranges[i];
      if (range < info->mRangeMin) range = 0;
      else if (range > info->mRangeMax) range = 1000;
      PointType pt;
      pt.x = cos(theta)*range;
      pt.y = sin(theta)*range;
      pt.z = 0;
      cloud->points.push_back(pt);
    }
    cloud->width = cloud->points.size();

    SensorData data;
    data.mPointSet.reset(new PointSet());
    data.mPointSet->mTimestamp = iMessage->utime;
    data.mPointSet->mMinRange = info->mRangeMin;
    data.mPointSet->mMaxRange = info->mRangeMax;
    data.mPointSet->mCloud = cloud;
    data.mChannel = iChannel;
    mImmediateBuffer.push(data);
    {
      boost::mutex::scoped_lock lock(mPoseUpdater.mMutex);
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
setLcm(const boost::shared_ptr<lcm::LCM>& iLcm) {
  mHelper->mLcm = iLcm;
  lcm_t* lcm = mHelper->mLcm->getUnderlyingLCM();
  mHelper->mBotParam = bot_param_get_global(lcm, 0);
  mHelper->mBotFrames.reset(new BotFramesWrapper());
  mHelper->mBotFrames->setLcm(mHelper->mLcm);
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

  // get min and max values from config
  char* lidarName =
    bot_param_get_planar_lidar_name_from_lcm_channel(mHelper->mBotParam,
                                                     iSensorChannel.c_str());
  char prefix[1024];
  bot_param_get_planar_lidar_prefix(NULL, lidarName, prefix, sizeof(prefix));
  std::string key = std::string(prefix) + ".max_range";
  double val;
  if (0 == bot_param_get_double(mHelper->mBotParam, key.c_str(), &val)) {
    info->mRangeMax = val;
  }
  key = std::string(prefix) + ".min_range";
  if (0 == bot_param_get_double(mHelper->mBotParam, key.c_str(), &val)) {
    info->mRangeMin = val;
  }
  free(lidarName);

  // subscribe to appropriate channel
  lcm::Subscription* sub = NULL;
  switch(iSensorType) {
  case SensorTypePlanarLidar:
    sub = mHelper->mLcm->subscribe
      (iSensorChannel, &SensorDataReceiver::Helper::onLidar, mHelper.get());
    break;
  case SensorTypePointCloud:
    sub = mHelper->mLcm->subscribe
      (iSensorChannel, &SensorDataReceiver::Helper::onCloud, mHelper.get());
    break;
  default:
    return false;
  }
  info->mSubscription = sub;
  {
    boost::mutex::scoped_lock lock(mHelper->mSubscriptionsMutex);
    mHelper->mSubscriptions[info->mSensorChannel] = info;
  }

  return true;
}

void SensorDataReceiver::
clearChannels() {
  boost::mutex::scoped_lock lock(mHelper->mSubscriptionsMutex);
  Helper::SubscriptionMap::const_iterator iter;
  for (iter = mHelper->mSubscriptions.begin();
       iter != mHelper->mSubscriptions.end(); ++iter) {
    mHelper->mLcm->unsubscribe(iter->second->mSubscription);
  }
  mHelper->mSubscriptions.clear();
}

bool SensorDataReceiver::
removeChannel(const std::string& iSensorChannel) {
  boost::mutex::scoped_lock lock(mHelper->mSubscriptionsMutex);
  Helper::SubscriptionMap::iterator item =
    mHelper->mSubscriptions.find(iSensorChannel);
  if (item == mHelper->mSubscriptions.end()) {
    return false;
  }
  mHelper->mLcm->unsubscribe(item->second->mSubscription);
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
    boost::thread(boost::ref(mHelper->mPoseUpdater));
  return true;
}

bool SensorDataReceiver::
stop() {
  if (!mHelper->mIsRunning) return false;
  mHelper->mIsRunning = false;
  try { mHelper->mPoseUpdater.mThread.join(); }
  catch (const boost::thread_interrupted&) {}
  return true;
}
