#ifndef _MultisenseLcm_Driver_hpp_
#define _MultisenseLcm_Driver_hpp_

#include <memory>
#include <string>

namespace lcm {
  class LCM;
}

class Imu;
class Laser;
class Camera;

class Driver {
public:
  Driver(const std::string& iIpAddress, const int iMtu);
  ~Driver();

  bool setPublishLcm(const std::shared_ptr<lcm::LCM>& iLcm);
  bool setSubscribeLcm(const std::shared_ptr<lcm::LCM>& iLcm);

  void enableImu(const bool iVal);

  void setCommandChannel(const std::string& iChannel);
  void setCameraChannel(const std::string& iChannel);
  void setLaserChannel(const std::string& iChannel);
  void setImuChannel(const std::string& iChannel);
  void setStateChannel(const std::string& iChannel);

  bool setLedState(const bool iFlash, const float iPercent);

  std::shared_ptr<Imu> getImu() const;
  std::shared_ptr<Laser> getLaser() const;
  std::shared_ptr<Camera> getCamera() const;

  bool start();
  bool stop();
  bool good() const;

private:
  struct Imp;
  std::shared_ptr<Imp> mImp;
};

#endif
