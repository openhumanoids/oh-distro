#ifndef _MultisenseLcm_Imu_hpp_
#define _MultisenseLcm_Imu_hpp_

#include <memory>
#include <string>

#include "Sensor.hpp"

class Imu : public Sensor {
public:
  Imu(crl::multisense::Channel* iChannel);
  ~Imu();

  bool start();
  bool stop();

private:
  struct Imp;
  std::shared_ptr<Imp> mImp;
};

#endif
