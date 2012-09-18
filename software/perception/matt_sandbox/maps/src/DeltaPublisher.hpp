#ifndef _DeltaPublisher_hpp_
#define _DeltaPublisher_hpp_

#include <boost/shared_ptr.hpp>
#include <string>

class MapManager;

namespace lcm {
  class LCM;
}

class DeltaPublisher {
public:
  DeltaPublisher();
  ~DeltaPublisher();

  void setLcm(boost::shared_ptr<lcm::LCM>& iLcm);
  void setManager(boost::shared_ptr<MapManager>& iManager);
  void setPublishInterval(const int iMilliseconds);
  void setPublishChannel(const std::string& iChannel);
  
  void operator()();
  bool start();
  bool stop();

protected: 
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<MapManager> mManager;
  std::string mChannel;
  int mPublishInterval;

  bool mIsRunning;
  int mNextMessageId;
};

#endif
