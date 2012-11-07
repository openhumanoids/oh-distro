#ifndef _DeltaPublisher_hpp_
#define _DeltaPublisher_hpp_

#include <boost/shared_ptr.hpp>
#include <string>
#include <unordered_map>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/message_ack_t.hpp>

class MapManager;

class DeltaPublisher {
public:
  DeltaPublisher();
  ~DeltaPublisher();

  void setLcm(boost::shared_ptr<lcm::LCM>& iLcm);
  void setManager(boost::shared_ptr<MapManager>& iManager);
  void setPublishInterval(const int iMilliseconds);
  void setParamsChannel(const std::string& iChannel);
  void setUpdateChannel(const std::string& iChannel);
  void setAckChannel(const std::string& iChannel);
  
  void operator()();
  bool start();
  bool stop();

protected:
  void onAck(const lcm::ReceiveBuffer* iBuf,
             const std::string& iChannel,
             const drc::message_ack_t* iMessage);

protected: 
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<MapManager> mManager;
  std::string mParamsChannel;
  std::string mUpdateChannel;
  std::string mAckChannel;
  int mPublishInterval;

  lcm::Subscription* mAckSubscription;
  std::unordered_map<int64_t,int64_t> mUnacknowledgedMessageIds;
  std::unordered_map<int64_t,bool> mSentMapIds;

  bool mIsRunning;
  int mNextMessageId;
};

#endif
