#ifndef _Clock_hpp_
#define _Clock_hpp_

#include <inttypes.h>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>

namespace drc {
 
class Clock {
protected:
  Clock();
  virtual ~Clock();

public:
  static Clock* getInstance();

  virtual void setLcm(const boost::shared_ptr<lcm::LCM>& iLcm);

  virtual void setChannel(const std::string& iChannelName);
  std::string getChannel() const;

  void setTimeoutInterval(const int iMilliseconds);

  virtual int64_t getCurrentTime() const = 0;

protected:
  boost::shared_ptr<lcm::LCM> mLcm;
  std::string mChannel;
  int mTimeoutInterval;
};
 
}

#endif
