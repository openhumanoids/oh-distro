#ifndef _MultisenseLcm_Laser_hpp_
#define _MultisenseLcm_Laser_hpp_

#include <memory>
#include <string>

#include "Sensor.hpp"

class Laser : public Sensor {
public:
  Laser(crl::multisense::Channel* iChannel);
  ~Laser();

  void setStateChannel(const std::string& iChannel);

  bool setSpindleSpeed(const float iRpm);

  bool start();
  bool stop();

private:
  struct Imp;
  std::shared_ptr<Imp> mImp;
};

#endif
