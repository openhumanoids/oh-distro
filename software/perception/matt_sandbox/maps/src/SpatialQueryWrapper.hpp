#ifndef _SpatialQueryWrapper_hpp_
#define _SpatialQueryWrapper_hpp_

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/raw_t.hpp>
#include <Eigen/Geometry>

class MapChunk;
class SpatialQuery;

class SpatialQueryWrapper {
public:
  SpatialQueryWrapper();
  ~SpatialQueryWrapper();

  void setLcm(boost::shared_ptr<lcm::LCM>& iLcm);
  void setMapChannel(const std::string& iChannel);

  void operator()();
  bool start();
  bool stop();

  bool lock();
  bool unlock();

  bool getClosest(const Eigen::Vector3d& iPoint,
                  Eigen::Vector3d& oPoint);
  bool getClosest(const Eigen::Vector3d& iPoint,
                  Eigen::Vector3d& oPoint, Eigen::Vector3d& oNormal);


protected:
  void onMap(const lcm::ReceiveBuffer* iBuf,
             const std::string& iChannel,
             const bot_core::raw_t* iMessage);

protected:
  boost::shared_ptr<MapChunk> mMap;
  boost::shared_ptr<SpatialQuery> mQuery;
  boost::shared_ptr<lcm::LCM> mLcm;
  std::string mMapChannel;

  lcm::Subscription* mMapSubscription;
  boost::mutex mQueryMutex;
  boost::mutex mMapMutex;
  boost::condition_variable mDataReady;
  boost::shared_ptr<SpatialQuery> mNewQuery;
  bool mNeedsUpdate;

  bool mIsRunning;
};

#endif
