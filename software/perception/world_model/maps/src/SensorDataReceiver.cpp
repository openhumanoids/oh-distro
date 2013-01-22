#include "SensorDataReceiver.hpp"

#include <pcl/io/io.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_util.h>

using namespace maps;

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
  clearChannels();
}

void SensorDataReceiver::
setBotParam(BotParam* iParam) {
  mBotParam = iParam;
  bot_frames_get_global(mLcm->getUnderlyingLCM(), mBotParam);
}

bool SensorDataReceiver::
addChannel(const std::string& iSensorChannel,
           const SensorType iSensorType,
           const std::string& iTransformFrom,
           const std::string& iTransformTo) {
  SubscriptionInfo info;
  info.mSensorChannel = iSensorChannel;
  info.mSensorType = iSensorType;
  info.mTransformFrom = iTransformFrom;
  info.mTransformTo = iTransformTo;

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

  info.mSubscription = sub;
  {
    boost::mutex::scoped_lock lock(mSubscriptionsMutex);
    mSubscriptions[info.mSensorChannel] = info;
  }

  return true;
}

void SensorDataReceiver::
clearChannels() {
  boost::mutex::scoped_lock lock(mSubscriptionsMutex);
  SubscriptionMap::const_iterator iter;
  for (iter = mSubscriptions.begin(); iter != mSubscriptions.end(); ++iter) {
    mLcm->unsubscribe(iter->second.mSubscription);
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
  mLcm->unsubscribe(item->second.mSubscription);
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
getPose(const std::string& iChannel, const int64_t iTimestamp,
        Eigen::Vector4f& oPosition, Eigen::Quaternionf& oOrientation) {
  BotFrames* frames = bot_frames_get_global(mLcm->getUnderlyingLCM(),
                                            mBotParam);
  boost::mutex::scoped_lock lock(mSubscriptionsMutex);
  SubscriptionMap::const_iterator item = mSubscriptions.find(iChannel);
  if (item == mSubscriptions.end()) {
    false;
  }
  BotTrans trans;
  int status =
    bot_frames_get_trans_with_utime(
        frames,
        item->second.mTransformFrom.c_str(),
        item->second.mTransformTo.c_str(),
        iTimestamp,
        &trans);
  if (0 == status) {
    std::cerr << "SensorDataReceiver: cannot get transform from " <<
      item->second.mTransformFrom << " to " << item->second.mTransformTo <<
      std::endl;
    return false;
  }

  oPosition = Eigen::Vector4f(trans.trans_vec[0], trans.trans_vec[1],
                              trans.trans_vec[2], 1);
  oOrientation = Eigen::Quaternionf(trans.rot_quat[0], trans.rot_quat[1],
                                    trans.rot_quat[2], trans.rot_quat[3]);
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

  if (!getPose(iChannel, iMessage->utime, newCloud->sensor_origin_,
               newCloud->sensor_orientation_)) {
    return;
  }
  maps::PointSet data;
  data.mTimestamp = iMessage->utime;
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

  // get min and max values from config
  double rangeMin(0), rangeMax(1e10), val;
  char* lidarName =
    bot_param_get_planar_lidar_name_from_lcm_channel(mBotParam,
                                                     iChannel.c_str());
  char prefix[1024];
  bot_param_get_planar_lidar_prefix(NULL, lidarName, prefix,
                                    sizeof(prefix));
  std::string key = std::string(prefix) + ".max_range";
  if (0 == bot_param_get_double(mBotParam, key.c_str(), &val)) {
    rangeMax = val;
  }
  key = std::string(prefix) + ".min_range";
  if (0 == bot_param_get_double(mBotParam, key.c_str(), &val)) {
    rangeMin = val;
  }
  free(lidarName);

  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.reserve(iMessage->nranges);
  for (int i = 0; i < iMessage->nranges; ++i) {
    double theta = iMessage->rad0 + i*iMessage->radstep;
    double range = iMessage->ranges[i];
    if (range < rangeMin) {
      range = 0;
    }
    else if (range > rangeMax) {
      range = 1000;
    }
    PointType pt;
    pt.x = cos(theta)*range;
    pt.y = sin(theta)*range;
    pt.z = 0;
    cloud->points.push_back(pt);
  }
  cloud->width = cloud->points.size();

  if (!getPose(iChannel, iMessage->utime, cloud->sensor_origin_,
               cloud->sensor_orientation_)) {
    return;
  }
  maps::PointSet data;
  data.mTimestamp = iMessage->utime;
  data.mCloud = cloud;
  mDataBuffer.push(data);
}
