#ifndef _MultisenseLcm_Camera_hpp_
#define _MultisenseLcm_Camera_hpp_

#include <memory>
#include <string>

#include "Sensor.hpp"

class Camera : public Sensor {
public:
  enum OutputMode {
    LeftDisparity=0,      // color left image + disparity
    LeftRight,            // mono left image + mono right image
    Left,                 // color left image only
    LeftRightDisparity,   // color left image + mono right image + disparity
  };

public:
  Camera(crl::multisense::Channel* iChannel);
  ~Camera();

  bool setOutputMode(const OutputMode iMode);
  bool setDesiredWidth(const int iPixels);
  bool setJpegQuality(const int iPercent);
  bool setZlibCompression(const int iLevel);

  bool setFrameRate(const float iFramesPerSecond);
  bool setGainFactor(const float iFactor);
  bool setExposureTime(const int iMicroSeconds);
  bool setAutoExposure(const bool iVal);

  bool start();
  bool stop();

private:
  struct Imp;
  std::shared_ptr<Imp> mImp;
};

#endif
