#ifndef _Clock_hpp_
#define _Clock_hpp_

#include <string>
#include <memory>

namespace lcm {
  class LCM;
}
typedef struct _lcm_t lcm_t;

namespace drc {
 
class Clock {
protected:
  struct Impl;

protected:
  Clock();
  virtual ~Clock();

public:
  static Clock* instance();

  void setLcm(const std::shared_ptr<lcm::LCM>& iLcm);
  void setLcm(const lcm_t* iLcm);

  void setChannel(const std::string& iChannelName);
  std::string getChannel() const;

  void setTimeoutInterval(const int iMilliseconds);
  void useTimeMessages(const bool iVal);
  void useRealTimeWhenInvalid(const bool iVal);
  void setVerbose(const bool iVal);

  int64_t getCurrentTime() const;
  int64_t getCurrentWallTime() const;

protected:
  std::shared_ptr<Impl> mImpl;
};
 
}

#endif
