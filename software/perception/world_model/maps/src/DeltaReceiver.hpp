#ifndef _DeltaReceiver_hpp_
#define _DeltaReceiver_hpp_

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_params_t.hpp>
#include <lcmtypes/drc/map_update_t.hpp>
#include <unordered_set>

#include "ThreadSafeQueue.hpp"

class MapManager;

class DeltaReceiver {
public:
  DeltaReceiver();
  ~DeltaReceiver();

  void setManager(boost::shared_ptr<MapManager>& iManager);
  void setLcm(boost::shared_ptr<lcm::LCM>& iLcm);
  void setParamsChannel(const std::string& iChannel);
  void setUpdateChannel(const std::string& iChannel);
  void setAckChannel(const std::string& iChannel);
  
  void operator()();
  bool start();
  bool stop();

protected:
  void onParams(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel,
                const drc::map_params_t* iMessage);
  void onUpdate(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel,
                const drc::map_update_t* iMessage);

  bool messageReceived(const int64_t iId);

protected:
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  std::string mParamsChannel;
  std::string mUpdateChannel;
  std::string mAckChannel;

  ThreadSafeQueue<drc::map_update_t> mDataBuffer;
  std::unordered_set<int64_t> mReceivedMessages;

  lcm::Subscription* mParamsSubscription;
  lcm::Subscription* mUpdateSubscription;

  bool mIsRunning;
};

#endif
