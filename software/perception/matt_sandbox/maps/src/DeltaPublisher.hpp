#ifndef _DeltaPublisher_hpp_
#define _DeltaPublisher_hpp_

#include <boost/shared_ptr.hpp>
#include <string>
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
  void setPublishChannel(const std::string& iChannel);
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
  std::string mDeltaChannel;
  std::string mAckChannel;
  int mPublishInterval;

  lcm::Subscription* mAckSubscription;

  bool mIsRunning;
  int mNextMessageId;
};

#endif
