#include "Sensor.hpp"

Sensor::
Sensor(crl::multisense::Channel* iChannel) {
  mChannel = iChannel;
}

Sensor::
~Sensor() {
}

void Sensor::
setOutputChannel(const std::string& iChannel) {
  mOutputChannel = iChannel;
}

void Sensor::
setPublishLcm(const std::shared_ptr<lcm::LCM>& iLcm) {
  mPublishLcm = iLcm;
}
