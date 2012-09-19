#ifndef _DeltaReceiver_hpp_
#define _DeltaReceiver_hpp_

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_delta_t.hpp>
#include <unordered_set>

#include "ThreadSafeQueue.hpp"

class MapManager;

class DeltaReceiver {
public:
  DeltaReceiver();
  ~DeltaReceiver();

  void setManager(boost::shared_ptr<MapManager>& iManager);
  void setLcm(boost::shared_ptr<lcm::LCM>& iLcm);
  void setDeltaChannel(const std::string& iChannel);
  void setAckChannel(const std::string& iChannel);
  
  void operator()();
  bool start();
  bool stop();

protected:
  void onDelta(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const drc::map_delta_t* iMessage);

  bool messageReceived(const int64_t iId);

protected:
  boost::shared_ptr<MapManager> mManager;
  boost::shared_ptr<lcm::LCM> mLcm;
  std::string mDeltaChannel;
  std::string mAckChannel;

  ThreadSafeQueue<drc::map_delta_t> mDataBuffer;
  std::unordered_set<int64_t> mReceivedMessages;

  lcm::Subscription* mDeltaSubscription;

  bool mIsRunning;
};

#endif
