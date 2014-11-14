#include "Driver.hpp"

#include "Imu.hpp"
#include "Laser.hpp"
#include "Camera.hpp"

#include <iostream>
#include <memory>

#include <multisense-lib/MultiSenseChannel.hh>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/multisense/command_t.hpp>


struct Driver::Imp {
  Driver* mDriver;
  crl::multisense::Channel* mChannel;

  std::shared_ptr<Imu> mImu;
  std::shared_ptr<Laser> mLaser;
  std::shared_ptr<Camera> mCamera;

  std::shared_ptr<lcm::LCM> mPublishLcm;
  std::shared_ptr<lcm::LCM> mSubscribeLcm;

  bool mEnableImu;
  std::string mCommandChannel;
  lcm::Subscription* mCommandSubscription;

  bool mIsRunning;

  Imp(Driver* iDriver) {
    mDriver = iDriver;
    mCommandSubscription = NULL;
    mIsRunning = false;
  }

  void commandHandler(const lcm::ReceiveBuffer* iBuf, 
                      const std::string& iChannel,
                      const multisense::command_t* iMessage) {
    std::cout << "command message received" << std::endl;
    mLaser->setSpindleSpeed(iMessage->rpm);
    if (iMessage->fps > 0) mCamera->setFrameRate(iMessage->fps);
    if (iMessage->gain > 0) mCamera->setGainFactor(iMessage->gain);
    if (iMessage->agc == 0) mCamera->setAutoExposure(false);
    if ((iMessage->exposure_us > 0) && (iMessage->agc <= 0)) {
      mCamera->setExposureTime(iMessage->exposure_us);
    }
    if (iMessage->agc > 0) mCamera->setAutoExposure(true);
    if ((iMessage->leds_flash >= 0) || (iMessage->leds_duty_cycle >= 0)) {
      mDriver->setLedState(iMessage->leds_flash>0, iMessage->leds_duty_cycle);
    }
  }

};

Driver::
Driver(const std::string& iIpAddress, const int iMtu) {
  mImp.reset(new Imp(this));

  mImp->mChannel = crl::multisense::Channel::Create(iIpAddress);
  if (mImp->mChannel == NULL) {
    std::cerr << "error: failed to create multisense channel" << std::endl;
    return;
  }
  std::cout << "successfully connected to device" << std::endl;
  auto status = mImp->mChannel->setMtu(iMtu);
  if (crl::multisense::Status_Ok != status) {
    std::cout << "warning: could not set mtu to " << iMtu << " (" <<
      mImp->mChannel->statusString(status) << ")" << std::endl;
  }

  setLedState(false, 0);

  mImp->mImu.reset(new Imu(mImp->mChannel));
  mImp->mLaser.reset(new Laser(mImp->mChannel));
  mImp->mCamera.reset(new Camera(mImp->mChannel));
}

Driver::
~Driver() {
  stop();
  mImp->mCamera.reset();
  mImp->mLaser.reset();
  mImp->mImu.reset();
  crl::multisense::Channel::Destroy(mImp->mChannel);
}

bool Driver::
setPublishLcm(const std::shared_ptr<lcm::LCM>& iLcm) {
  mImp->mPublishLcm = iLcm;
  mImp->mImu->setPublishLcm(iLcm);
  mImp->mLaser->setPublishLcm(iLcm);
  mImp->mCamera->setPublishLcm(iLcm);
  return true;
}

bool Driver::
setSubscribeLcm(const std::shared_ptr<lcm::LCM>& iLcm) {
  mImp->mSubscribeLcm = iLcm;
  return true;
}

void Driver::
enableImu(const bool iVal) {
  mImp->mEnableImu = iVal;
}

void Driver::
setCommandChannel(const std::string& iChannel) {
  mImp->mCommandChannel = iChannel;
}

void Driver::
setCameraChannel(const std::string& iChannel) {
  mImp->mCamera->setOutputChannel(iChannel);
}

void Driver::
setLaserChannel(const std::string& iChannel) {
  mImp->mLaser->setOutputChannel(iChannel);
}

void Driver::
setImuChannel(const std::string& iChannel) {
  mImp->mImu->setOutputChannel(iChannel);
}

void Driver::
setStateChannel(const std::string& iChannel) {
  mImp->mLaser->setStateChannel(iChannel);
}

bool Driver::
setLedState(const bool iFlash, const float iPercent) {
  crl::multisense::lighting::Config config;
  config.setFlash(iFlash);
  config.setDutyCycle(iPercent);
  auto status = mImp->mChannel->setLightingConfig(config);
  if (crl::multisense::Status_Ok != status) {
    std::cerr << "failed to set led config ( " <<
      mImp->mChannel->statusString(status) << ")" << std::endl;
    return false;
  }
  return true;
}


std::shared_ptr<Imu> Driver::
getImu() const {
  return mImp->mImu;
}

std::shared_ptr<Laser> Driver::
getLaser() const {
  return mImp->mLaser;
}

std::shared_ptr<Camera> Driver::
getCamera() const {
  return mImp->mCamera;
}


bool Driver::
start() {
  if (mImp->mIsRunning) return false;
  if (mImp->mChannel == NULL) return false;
  mImp->mIsRunning = true;

  mImp->mCommandSubscription =
    mImp->mSubscribeLcm->subscribe(mImp->mCommandChannel,
                                   &Imp::commandHandler, mImp.get());

  if (mImp->mEnableImu) {
    if (!mImp->mImu->start()) return false;
  }
  if (!mImp->mLaser->start()) return false;
  if (!mImp->mCamera->start()) return false;

  return true;
}

bool Driver::
stop() {
  if (!mImp->mIsRunning) return false;
  mImp->mIsRunning = false;

  if (mImp->mCommandSubscription != NULL) {
    mImp->mSubscribeLcm->unsubscribe(mImp->mCommandSubscription);
  }

  mImp->mCamera->stop();
  mImp->mLaser->stop();
  mImp->mImu->stop();

  setLedState(false, 0);

  std::cout << "driver stopped" << std::endl;

  return true;
}

bool Driver::
good() const {
  return mImp->mChannel!=NULL;
}
