#ifndef _MultisenseLcm_Sensor_hpp_
#define _MultisenseLcm_Sensor_hpp_

#include <string>
#include <memory>

namespace lcm {
  class LCM;
}

namespace crl {
  namespace multisense {
    class Channel;
  }
}

class Sensor {
public:
  Sensor(crl::multisense::Channel* iChannel);
  virtual ~Sensor();

  void setOutputChannel(const std::string& iChannel);
  void setPublishLcm(const std::shared_ptr<lcm::LCM>& iLcm);

  virtual bool start() = 0;
  virtual bool stop() = 0;

protected:
  std::string mOutputChannel;
  std::shared_ptr<lcm::LCM> mPublishLcm;
  crl::multisense::Channel* mChannel;
};

#endif
