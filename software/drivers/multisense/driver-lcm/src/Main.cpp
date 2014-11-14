#include <iostream>
#include <csignal>

#include <ConciseArgs>
#include <lcm/lcm-cpp.hpp>

#include "Driver.hpp"
#include "Laser.hpp"
#include "Camera.hpp"

std::shared_ptr<Driver> gDriver;

// handle interrupt signal and shut down cleanly
void signalHandler(const int iSignal) {
  static bool signalCaught = false;
  if (signalCaught) return;
  signalCaught = true;
  std::cout << "stopping driver..." << std::endl;
  gDriver.reset();
  exit(1);
};


int main(const int iArgc, const char** iArgv) {

  // program options
  int outputMode = 0;
  int jpegQuality = 75;
  int zlibCompression = 1;
  bool useImu = false;
  int desiredWidth = 1024;
  std::string ipAddress("10.66.171.21");
  int mtu = 7200;
  std::string cameraChannel("CAMERA");
  std::string laserChannel("SCAN");
  std::string imuChannel("");
  std::string stateChannel("MULTISENSE_STATE");
  std::string commandChannel("MULTISENSE_COMMAND");
  std::string lcmUrl("");

  // parse command line arguments
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(ipAddress,"a","ip","sensor ip address");
  opt.add(mtu,"m","mtu","sensor mtu");
  opt.add(desiredWidth,"w","width","desired image width");
  opt.add(outputMode,"o","output-mode",
          "0=L+disp, 1=L+R, 2=L, 3=L+R+disp");
  opt.add(jpegQuality,"j","jpeg-quality",
          "jpeg quality (1-100), 100=no compression");
  opt.add(zlibCompression,"z","zlib",
          "disparity compression (0-9), 0=no compression");
  opt.add(lcmUrl,"u","lcm-url","url of lcm community");
  opt.add(cameraChannel,"c","camera-channel","camera output lcm channel");
  opt.add(laserChannel,"l","laser-channel","laser output lcm channel");
  opt.add(imuChannel,"i","imu-channel","imu output lcm channel");
  opt.add(stateChannel,"s","state-channel","sensor state output lcm channel");
  opt.add(commandChannel,"k","command-channel",
          "sensor command input lcm channel");
  opt.parse();

  // echo settings
  std::cout << "===== options set =====" << std::endl;
  std::cout << "  sensor ip: " << ipAddress << std::endl;
  std::cout << "  sensor mtu: " << mtu << std::endl;
  std::cout << "  requested image width: " << desiredWidth << std::endl;
  std::cout << "  output mode: ";
  switch (outputMode) {
  case Camera::OutputMode::LeftDisparity: std::cout << "L+disp"; break;
  case Camera::OutputMode::LeftRight: std::cout << "L+R"; break;
  case Camera::OutputMode::Left: std::cout << "L"; break;
  case Camera::OutputMode::LeftRightDisparity: std::cout << "L+R+disp"; break;
  default: std::cout << "undefined"; break;
  }
  std::cout << std::endl;
  std::cout << "  jpeg quality: " << jpegQuality << std::endl;
  if (zlibCompression==0) std::cout << "  zlib compression: off" << std::endl;
  else std::cout << "  zlib compression: " << zlibCompression << std::endl;
  if (lcmUrl.length()>0) std::cout << "  lcm url: " << lcmUrl << std::endl;
  else std::cout << "  lcm url: default" << std::endl;
  std::cout << "  camera output channel: " << cameraChannel << std::endl;
  std::cout << "  laser output channel: " << laserChannel << std::endl;
  if (useImu) std::cout << "  imu output channel: " << imuChannel << std::endl;
  else std::cout << "  imu disabled" << std::endl;
  std::cout << "  state output channel: " << stateChannel << std::endl;
  std::cout << "  command input channel: " << commandChannel << std::endl;
  std::cout << "===== end options =====" << std::endl << std::endl;

  // set up lcm
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM(lcmUrl));
  if (!lcm->good()) {
    std::cerr << "error: cannot create lcm instance on " << lcmUrl << std::endl;
    return -1;
  }

  // instantiate driver
  gDriver.reset(new Driver(ipAddress, mtu));
  if (!gDriver->good()) {
    std::cerr << "error: driver cannot start" << std::endl;
    return -1;
  }

  // set up interrupt handler
  std::signal(SIGINT, signalHandler);

  // set channels
  gDriver->setSubscribeLcm(lcm);
  gDriver->setPublishLcm(lcm);
  gDriver->setImuChannel(imuChannel);
  gDriver->setLaserChannel(laserChannel);
  gDriver->setCameraChannel(cameraChannel);
  gDriver->setStateChannel(stateChannel);
  gDriver->setCommandChannel(commandChannel);
  gDriver->enableImu(useImu);

  // set initial parameters
  gDriver->getCamera()->setDesiredWidth(desiredWidth);
  gDriver->getCamera()->setJpegQuality(jpegQuality);
  gDriver->getCamera()->setZlibCompression(zlibCompression);
  gDriver->getCamera()->setOutputMode((Camera::OutputMode)outputMode);

  // start driver
  if (!gDriver->start()) {
    std::cerr << "error: could not start driver" << std::endl;
    return -1;
  }
  std::cout << "driver started successfully" << std::endl;

  // lcm handle loop (sensors run in background threads)
  while (true) {
    lcm->handle();
  }

  return 1;
}
