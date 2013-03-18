#ifndef _Clock_hpp_
#define _Clock_hpp_

#include <inttypes.h>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>
#include <bot_core/timestamp.h>

namespace drc {
 
class Clock {
protected:
  Clock();
  virtual ~Clock();

public:
  static Clock* instance();

  virtual void setLcm(const boost::shared_ptr<lcm::LCM>& iLcm);
  virtual void setLcm(const lcm_t* iLcm);

  virtual void setChannel(const std::string& iChannelName);
  std::string getChannel() const;

  void setTimeoutInterval(const int iMilliseconds);
  void useTimeMessages(const bool iVal);
  void useRealTimeWhenInvalid(const bool iVal);
  void setVerbose(const bool iVal);

  virtual int64_t getCurrentTime() const = 0;

  virtual int64_t getCurrentWallTime() const = 0;

protected:
  virtual void update() = 0;

protected:
  boost::shared_ptr<lcm::LCM> mLcm;
  std::string mChannel;
  int mTimeoutInterval;
  bool mUseTimeMessages;
  bool mUseRealTimeWhenInvalid;
  bool mVerbose;
};
 
}

#endif
