#include "SensorDataReceiver.hpp"

#include "BotFramesWrapper.hpp"

#include <pcl/io/io.h>
#include <bot_param/param_util.h>

using namespace maps;

struct SensorDataReceiver::SubscriptionInfo {
  std::string mSensorChannel;
  SensorType mSensorType;
  std::string mTransformFrom;
  std::string mTransformTo;
  float mRangeMin;
  float mRangeMax;
  lcm::Subscription* mSubscription;
  typedef boost::shared_ptr<SubscriptionInfo> Ptr;
};

struct SensorDataReceiver::BotStructures {
  BotParam* mParam;
};


SensorDataReceiver::
SensorDataReceiver() {
  mIsRunning = false;
  setMaxBufferSize(100);
}

SensorDataReceiver::
~SensorDataReceiver() {
  clearChannels();
}

void SensorDataReceiver::
setLcm(boost::shared_ptr<lcm::LCM>& iLcm) {
  mLcm = iLcm;
  lcm_t* lcm = mLcm->getUnderlyingLCM();
  mBotStructures.reset(new BotStructures());
  mBotStructures->mParam = bot_param_get_global(lcm, 0);
  mBotFrames.reset(new BotFramesWrapper());
  mBotFrames->setLcm(mLcm);
  clearChannels();
}

bool SensorDataReceiver::
addChannel(const std::string& iSensorChannel,
           const SensorType iSensorType,
           const std::string& iTransformFrom,
           const std::string& iTransformTo) {
  SubscriptionInfo::Ptr info(new SubscriptionInfo());
  info->mSensorChannel = iSensorChannel;
  info->mSensorType = iSensorType;
  info->mTransformFrom = iTransformFrom;
  info->mTransformTo = iTransformTo;
  info->mRangeMin = 0;
  info->mRangeMax = 1e10;

  // get min and max values from config
  char* lidarName =
    bot_param_get_planar_lidar_name_from_lcm_channel(mBotStructures->mParam,
                                                     iSensorChannel.c_str());
  char prefix[1024];
  bot_param_get_planar_lidar_prefix(NULL, lidarName, prefix, sizeof(prefix));
  std::string key = std::string(prefix) + ".max_range";
  double val;
  if (0 == bot_param_get_double(mBotStructures->mParam, key.c_str(), &val)) {
    info->mRangeMax = val;
  }
  key = std::string(prefix) + ".min_range";
  if (0 == bot_param_get_double(mBotStructures->mParam, key.c_str(), &val)) {
    info->mRangeMin = val;
  }
  free(lidarName);

  // subscribe to appropriate channel
  lcm::Subscription* sub = NULL;
  switch(iSensorType) {
  case SensorTypePlanarLidar:
    sub = mLcm->subscribe(iSensorChannel,
                          &SensorDataReceiver::onLidar, this);
    break;
  case SensorTypePointCloud:
    sub = mLcm->subscribe(iSensorChannel,
                          &SensorDataReceiver::onPointCloud, this);
    break;
  default:
    return false;
  }
  info->mSubscription = sub;
  {
    boost::mutex::scoped_lock lock(mSubscriptionsMutex);
    mSubscriptions[info->mSensorChannel] = info;
  }

  return true;
}

void SensorDataReceiver::
clearChannels() {
  boost::mutex::scoped_lock lock(mSubscriptionsMutex);
  SubscriptionMap::const_iterator iter;
  for (iter = mSubscriptions.begin(); iter != mSubscriptions.end(); ++iter) {
    mLcm->unsubscribe(iter->second->mSubscription);
  }
  mSubscriptions.clear();
}

bool SensorDataReceiver::
removeChannel(const std::string& iSensorChannel) {
  boost::mutex::scoped_lock lock(mSubscriptionsMutex);
  SubscriptionMap::iterator item = mSubscriptions.find(iSensorChannel);
  if (item == mSubscriptions.end()) {
    return false;
  }
  mLcm->unsubscribe(item->second->mSubscription);
  mSubscriptions.erase(item);
  return true;
}

void SensorDataReceiver::
setMaxBufferSize(const int iSize) {
  mDataBuffer.setMaxSize(iSize);
}

bool SensorDataReceiver::
pop(maps::PointSet& oData) {
  return mDataBuffer.pop(oData);
}

bool SensorDataReceiver::
waitForData(maps::PointSet& oData) {
  return mDataBuffer.waitForData(oData);
}

bool SensorDataReceiver::
start() {
  if (mIsRunning) {
    return false;
  }
  mIsRunning = true;
  return true;
}

bool SensorDataReceiver::
stop() {
  if (!mIsRunning) {
    return false;
  }
  mIsRunning = false;
  return true;
}

bool SensorDataReceiver::
getPose(const SubscriptionInfo::Ptr& iInfo, const int64_t iTimestamp,
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


void SensorDataReceiver::
onPointCloud(const lcm::ReceiveBuffer* iBuf,
             const std::string& iChannel,
             const drc::pointcloud2_t* iMessage) {

  if (!mIsRunning) {
    return;
  }

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

  SubscriptionMap::const_iterator item;
  {
    boost::mutex::scoped_lock lock(mSubscriptionsMutex);
    item = mSubscriptions.find(iChannel);
    if (item == mSubscriptions.end()) {
      return;
    }
  }
  if (!getPose(item->second, iMessage->utime, newCloud->sensor_origin_,
               newCloud->sensor_orientation_)) {
    return;
  }

  maps::PointSet data;
  data.mTimestamp = iMessage->utime;
  data.mMaxRange = 1e10;
  data.mCloud = newCloud;
  mDataBuffer.push(data);
}

void SensorDataReceiver::
onLidar(const lcm::ReceiveBuffer* iBuf,
        const std::string& iChannel,
        const bot_core::planar_lidar_t* iMessage) {

  if (!mIsRunning) {
    return;
  }

  SubscriptionMap::const_iterator item;
  {
    boost::mutex::scoped_lock lock(mSubscriptionsMutex);
    item = mSubscriptions.find(iChannel);
    if (item == mSubscriptions.end()) {
      return;
    }
  }

  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.reserve(iMessage->nranges);
  for (int i = 0; i < iMessage->nranges; ++i) {
    double theta = iMessage->rad0 + i*iMessage->radstep;
    double range = iMessage->ranges[i];
    if (range < item->second->mRangeMin) {
      range = 0;
    }
    else if (range > item->second->mRangeMax) {
      range = 1000;
    }
    PointType pt;
    pt.x = cos(theta)*range;
    pt.y = sin(theta)*range;
    pt.z = 0;
    cloud->points.push_back(pt);
  }
  cloud->width = cloud->points.size();

  if (!getPose(item->second, iMessage->utime, cloud->sensor_origin_,
               cloud->sensor_orientation_)) {
    return;
  }
  maps::PointSet data;
  data.mTimestamp = iMessage->utime;
  data.mMaxRange = item->second->mRangeMax;
  data.mCloud = cloud;
  mDataBuffer.push(data);
}
