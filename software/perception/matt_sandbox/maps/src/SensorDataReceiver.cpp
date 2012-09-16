#include "SensorDataReceiver.hpp"

#include <pcl/io/io.h>
#include <bot_frames/bot_frames.h>

SensorDataReceiver::
SensorDataReceiver() {
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
  mMaxBufferSize = iSize;
  boost::mutex::scoped_lock lock(mBufferMutex);
  while (mDataBuffer.size() > mMaxBufferSize) {
    mDataBuffer.pop_front();
  }
}

int SensorDataReceiver::
getBufferSize() const {
  return mDataBuffer.size();
}

bool SensorDataReceiver::
pop(PointCloudWithPose& oData) {
  boost::mutex::scoped_lock lock(mBufferMutex);
  if (mDataBuffer.size() == 0) {
    return false;
  }
  oData = mDataBuffer.front();
  mDataBuffer.pop_front();
  return true;
}

bool SensorDataReceiver::
waitForData(PointCloudWithPose& oData) {
  boost::mutex::scoped_lock lock(mBufferMutex);
  while (mDataBuffer.empty()) {
    mBufferCondition.wait(lock);
  }
  oData = mDataBuffer.front();
  mDataBuffer.pop_front();
  return true;
}

void SensorDataReceiver::
onPointCloud(const lcm::ReceiveBuffer* iBuf,
             const std::string& iChannel,
             const drc::pointcloud2_t* iMessage) {

  pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
  pointCloud.width = iMessage->width;
  pointCloud.height = iMessage->height;
  pointCloud.points.resize(iMessage->width * iMessage->height);
  pointCloud.is_dense = false;
  uint8_t* cloudData = reinterpret_cast<uint8_t*>(&pointCloud.points[0]);
  const uint8_t* messageData =
    reinterpret_cast<const uint8_t*>(&iMessage->data[0]);
  memcpy(cloudData, messageData, iMessage->data_nbytes);
  PointCloud::Ptr newCloud(new PointCloud());
  pcl::copyPointCloud(pointCloud, *newCloud);

  BotFrames* frames = bot_frames_get_global(mLcm->getUnderlyingLCM(),
                                            mBotParam);
  double matx[16];
  {
    boost::mutex::scoped_lock lock(mSubscriptionsMutex);
    SubscriptionMap::const_iterator item = mSubscriptions.find(iChannel);
    if (item == mSubscriptions.end()) {
      return;
    }
    int status =
      bot_frames_get_trans_mat_4x4_with_utime(
        frames,
        item->second.mTransformFrom.c_str(),
        item->second.mTransformTo.c_str(),
        iMessage->utime,
        matx);
    if (0 == status) {
      std::cerr << "SensorDataReceiver: cannot get transform from " <<
        item->second.mTransformFrom << " to " << item->second.mTransformTo <<
        std::endl;
      return;
    }
  }
  Eigen::Isometry3d pose;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      pose(i,j) = matx[i*4+j];
    }
  }
  PointCloudWithPose data(iMessage->utime, newCloud, pose);
  {
    boost::mutex::scoped_lock lock(mBufferMutex);
    while (mDataBuffer.size() >= mMaxBufferSize) {
      mDataBuffer.pop_front();
      std::cout << "SensorDataReceiver: discarding item from buffer " <<
        mDataBuffer.size() << " " << mMaxBufferSize << std::endl;
    }
    mDataBuffer.push_back(data);
    lock.unlock();
    mBufferCondition.notify_one();
  }
}

void SensorDataReceiver::
onLidar(const lcm::ReceiveBuffer* iBuf,
        const std::string& iChannel,
        const bot_core::planar_lidar_t* iMessage) {

  PointCloud::Ptr cloud(new PointCloud());
  cloud->width = iMessage->nranges;
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(iMessage->nranges);
  for (int i = 0; i < iMessage->nranges; ++i) {
    double theta = iMessage->rad0 + i*iMessage->radstep;
    double range = iMessage->ranges[i];
    cloud->points[i] =
      PointCloud::PointType(cos(theta)*range, sin(theta)*range, 0);
  }

  BotFrames* frames = bot_frames_get_global(mLcm->getUnderlyingLCM(),
                                            mBotParam);
  double matx[16];
  {
    boost::mutex::scoped_lock lock(mSubscriptionsMutex);
    SubscriptionMap::const_iterator item = mSubscriptions.find(iChannel);
    if (item == mSubscriptions.end()) {
      return;
    }
    int status =
      bot_frames_get_trans_mat_4x4_with_utime(
        frames,
        item->second.mTransformFrom.c_str(),
        item->second.mTransformTo.c_str(),
        iMessage->utime,
        matx);
    if (0 == status) {
      std::cerr << "SensorDataReceiver: cannot get transform from " <<
        item->second.mTransformFrom << " to " << item->second.mTransformTo <<
        std::endl;
      return;
    }
  }
  Eigen::Isometry3d pose;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      pose(i,j) = matx[i*4+j];
    }
  }
  PointCloudWithPose data(iMessage->utime, cloud, pose);
  {
    boost::mutex::scoped_lock lock(mBufferMutex);
    while (mDataBuffer.size() >= mMaxBufferSize) {
      mDataBuffer.pop_front();
      std::cout << "SensorDataReceiver: discarding item from buffer " <<
        mDataBuffer.size() << " " << mMaxBufferSize << std::endl;
    }
    mDataBuffer.push_back(data);
    lock.unlock();
    mBufferCondition.notify_one();
  }
}
