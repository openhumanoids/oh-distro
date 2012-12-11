#ifndef _SensorDataReceiver_hpp_
#define _SensorDataReceiver_hpp_

#include <unordered_map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/drc/pointcloud2_t.hpp>
#include <bot_param/param_client.h>

#include "MapTypes.hpp"
#include "ThreadSafeQueue.hpp"

class SensorDataReceiver {
public:
  enum SensorType {
    SensorTypePlanarLidar,
    SensorTypePointCloud
  };

  struct PointCloudWithPose {
    int64_t mTimestamp;
    maptypes::PointCloud::Ptr mPointCloud;
    Eigen::Isometry3d mPose;

    PointCloudWithPose() {}
    PointCloudWithPose(const int64_t iTime,
                       const maptypes::PointCloud::Ptr& iCloud,
                       const Eigen::Isometry3d& iPose) {
      mTimestamp = iTime;
      mPointCloud = iCloud;
      mPose = iPose;
    }
  };

protected:
  struct SubscriptionInfo {
    std::string mSensorChannel;
    SensorType mSensorType;
    std::string mTransformFrom;
    std::string mTransformTo;
    lcm::Subscription* mSubscription;
  };

  typedef std::unordered_map<std::string, SubscriptionInfo> SubscriptionMap;

public:
  SensorDataReceiver();
  ~SensorDataReceiver();

  void setLcm(boost::shared_ptr<lcm::LCM>& iLcm);
  void setBotParam(BotParam* iParam);
  bool addChannel(const std::string& iSensorChannel,
                  const SensorType iSensorType,
                  const std::string& iTransformFrom,
                  const std::string& iTransformTo);
  void clearChannels();
  bool removeChannel(const std::string& iSensorChannel);

  void setMaxBufferSize(const int iSize);
  bool pop(PointCloudWithPose& oData);
  bool waitForData(PointCloudWithPose& oData);

  bool start();
  bool stop();

protected:

  bool getPose(const std::string& iChannel, const int64_t iTimestamp,
               Eigen::Isometry3d& oPose);


  // message handlers
  void onPointCloud(const lcm::ReceiveBuffer* iBuf,
                    const std::string& iChannel,
                    const drc::pointcloud2_t* iMessage);
  void onLidar(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const bot_core::planar_lidar_t* iMessage);

protected:
  boost::shared_ptr<lcm::LCM> mLcm;
  BotParam* mBotParam;
  SubscriptionMap mSubscriptions;
  bool mIsRunning;

  ThreadSafeQueue<PointCloudWithPose> mDataBuffer;
  boost::mutex mBufferMutex;
  boost::mutex mSubscriptionsMutex;
  boost::condition_variable mBufferCondition;
};

#endif
