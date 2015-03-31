#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <bot_core/timestamp.h>
#include <jpeg-utils/jpeg-utils.h>
#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>
#include <flycapture/FlyCapture2.h>
#include <lcmtypes/bot_core/image_t.hpp>

struct State {
  std::string mPublishChannel;
  int mCompressionQuality;
  int mFrameRate;
  int mRotation;
  int mEqualizeRadius;
  bool mTrigger;
  float mInitSeconds;
  float mDelaySeconds;
  int64_t mDesiredSerial;
  std::string mDesiredIpAddress;

  bool mShouldCompress;
  int mPeriodMs;
  bool mIsRunning;

  bot_timestamp_sync_state_t* mBotSync;
  std::thread mCaptureThread;
  std::shared_ptr<lcm::LCM> mLcm;
  FlyCapture2::Camera mCamera;
  FlyCapture2::PGRGuid mCameraUid;

  cv::Mat mEqualizeMask;

  State() {
    mPublishChannel = "DUMMY_CAMERA_CHANNEL";
    mCompressionQuality = 100;
    mFrameRate = 0;
    mRotation = 0;
    mTrigger = false;
    mInitSeconds = 3;
    mDelaySeconds = 0;
    mDesiredSerial = -1;
    mEqualizeRadius = -1;
    mLcm.reset(new lcm::LCM());
    mIsRunning = false;
    mBotSync = bot_timestamp_sync_init(8000, 128*8000, 1.001);
  }

  ~State() {
    bot_timestamp_sync_free(mBotSync);
  }

  bool start() {
    mShouldCompress = (mCompressionQuality < 100);
    mPeriodMs = (mFrameRate == 0 ? 0 : 1000/mFrameRate);

    // wait first
    std::cout << "Waiting for " << mDelaySeconds << " seconds..." << std::flush;
    std::this_thread::sleep_for
      (std::chrono::milliseconds((int)(1000*mDelaySeconds)));
    std::cout << "done." << std::endl;

    // connect to camera
    std::cout << "Connecting to camera..." << std::flush;
    FlyCapture2::Error error;

    // try to find by serial number
    if (mDesiredSerial >= 0) {
      if (!findCameraBySerial()) return false;
      error = mCamera.Connect(&mCameraUid);
    }

    // try to find by ip address
    else if (mDesiredIpAddress.length() > 0) {
      int curComponent = 0;
      std::stringstream ss(mDesiredIpAddress);
      std::string token;
      FlyCapture2::IPAddress address;
      while (std::getline(ss, token, '.') && (curComponent <= 3)) {
	int component = stoi(token);
	address.octets[curComponent] = component;
	++curComponent;
      }
      FlyCapture2::BusManager busManager;
      error = busManager.GetCameraFromIPAddress(address, &mCameraUid);
      if (error != FlyCapture2::PGRERROR_OK) {
	std::cout << "failed to find camera on ip address " <<
	  mDesiredIpAddress << std::endl;
	return false;
      }
      error = mCamera.Connect(&mCameraUid);
    }

    // fallback to default camera
    else error = mCamera.Connect(NULL);
    if (error != FlyCapture2::PGRERROR_OK) {
      std::cout << "failed" << std::endl;
      return false;
    }
    std::cout << "done" << std::endl;

    // stop capture if in bad state
    error = mCamera.StopCapture();

    // get camera info
    FlyCapture2::CameraInfo camInfo;
    std::cout << "Getting camera information..." << std::flush;
    error = mCamera.GetCameraInfo(&camInfo);
    if (error != FlyCapture2::PGRERROR_OK) {
      std::cout << "failed" << std::endl;
      return false;
    }
    std::cout << "done" << std::endl;
    std::cout << "Camera information: " << camInfo.vendorName << " " <<
      camInfo.modelName << " " << camInfo.serialNumber << std::endl;

    // set configuration parameters
    if (!setConfig()) return false;

    // set properties
    if (!setProperties()) return false;

    // set trigger mode
    if (!setTriggerMode()) return false;

    // start capture
    std::cout << "Starting capture..." << std::flush;
    error = mCamera.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK) {
      std::cout << "failed: " << error.GetDescription() << std::endl;
      return false;
    }
    std::cout << "done" << std::endl;

    // stream some frames to flush out system
    if (!streamFrames(mInitSeconds*1000)) return false;

    // launch capture thread
    mIsRunning = true;
    mCaptureThread = std::thread(std::ref(*this));

    // subscribe to control messages
    // TODO

    return true;
  }

  void createEqualizeMask(const int iRows, const int iCols) {
    int xCenter = iCols/2;
    int yCenter = iRows/2;
    int r2 = mEqualizeRadius*mEqualizeRadius;
    mEqualizeMask = cv::Mat(iRows, iCols, CV_8UC1);
    for (int i = 0; i < mEqualizeMask.rows; ++i) {
      uint8_t* outPtr = mEqualizeMask.ptr<uint8_t>(i);
      int y = i-yCenter;
      int y2 = y*y;
      for (int j = 0; j < mEqualizeMask.cols; ++j, ++outPtr) {
        int x = j-xCenter;
        int x2 = x*x;
        *outPtr = (x2+y2 <= r2 ? 255 : 0);
      }
    }
  }

  void equalize(cv::Mat& ioImage) {
    // create mask if it does not already exist
    if (mEqualizeMask.rows == 0) {
      createEqualizeMask(ioImage.rows, ioImage.cols);
    }

    // create and initialize histograms
    const int numBins = 256;
    int hist[3][numBins];
    for (int i = 0; i < 3; ++i) {
      std::fill(hist[i], hist[i]+numBins, 0);
    }

    // compute histograms
    for (int i = 0; i < ioImage.rows; ++i) {
      uint8_t* imgPtr = ioImage.ptr<uint8_t>(i);
      uint8_t* maskPtr = mEqualizeMask.ptr<uint8_t>(i);
      for (int j = 0; j < ioImage.cols; ++j, ++maskPtr) {
        for (int k = 0; k < ioImage.channels(); ++k, ++imgPtr) {
          if (*maskPtr > 0) ++hist[k][*imgPtr];
        }
      }
    }

    // compute cdfs
    int cdf[3][numBins];
    int total[3];
    for (int i = 0; i < 3; ++i) {
      cdf[i][0] = hist[i][0];
      for (int j = 1; j < numBins; ++j) {
        cdf[i][j] = cdf[i][j-1] + hist[i][j];
      }
      total[i] = cdf[i][numBins-1];
    }

    // desired cdf
    float desired[3][numBins];
    for (int i = 0; i < 3; ++i) {
      desired[i][0] = 0.5*total[i]/numBins;
      for (int j = 1; j < numBins; ++j) {
        desired[i][j] = desired[i][j-1] + (float)total[i]/numBins;
      }
    }

    // create and initialize lookup tables
    int lut[3][numBins];
    for (int i = 0; i < 3; ++i) {
      std::fill(lut[i], lut[i] + numBins, 0);
    }

    // generate lookup tables
    for (int i = 0; i < 3; ++i) {
      int idx = 0;
      for (int j = 0; j < numBins; ++j) {
        while ((idx < numBins-1) && (desired[i][idx] < cdf[i][j])) ++idx;
        lut[i][j] = idx;
      }
    }

    // apply lookup table
    for (int i = 0; i < ioImage.rows; ++i) {
      uint8_t* imgPtr = ioImage.ptr<uint8_t>(i);
      for (int j = 0; j < ioImage.cols; ++j) {
        for (int k = 0; k < ioImage.channels(); ++k, ++imgPtr) {
          *imgPtr = lut[k][*imgPtr];
        }
      }
    }
  }

  void convert(const FlyCapture2::Image& iImage, cv::Mat& oImage) {
    oImage = cv::Mat(iImage.GetRows(), iImage.GetCols(), CV_8UC3,
                     iImage.GetData(), iImage.GetStride());
    switch (mRotation) {
    case 90: cv::transpose(oImage, oImage); cv::flip(oImage, oImage, 1); break;
    case 270: cv::transpose(oImage, oImage); cv::flip(oImage, oImage, 0); break;
    case 180: cv::flip(oImage, oImage, -1); break;
    default: break;
    }
    if (mEqualizeRadius > 0) {
      equalize(oImage);
    }
  }

  void convert(const cv::Mat& iImage, bot_core::image_t& oImage) {
    oImage.width = iImage.cols;
    oImage.height = iImage.rows;
    oImage.row_stride = iImage.step;
    oImage.nmetadata = 0;

    // compress if necessary
    uint8_t* data = iImage.data;
    oImage.size = oImage.height*oImage.row_stride;
    if (mShouldCompress) {
      std::vector<uint8_t> dest(oImage.size);
      jpeg_compress_8u_rgb(data, oImage.width, oImage.height, oImage.row_stride,
                           dest.data(), &oImage.size, mCompressionQuality);
      oImage.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
      oImage.data.resize(oImage.size);
      std::copy(dest.data(), dest.data()+oImage.size, oImage.data.begin());
    }

    // otherwise just set raw bytes
    else {
      oImage.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
      oImage.data.resize(oImage.size);
      std::copy(data, data + oImage.size, oImage.data.begin());
    }
  }

  int64_t getImageTime(const FlyCapture2::Image& iImage) {
    FlyCapture2::TimeStamp time = iImage.GetTimeStamp();
    int64_t cameraStamp = time.cycleSeconds*8000 + time.cycleCount;
    int64_t wallStamp = bot_timestamp_now();
    int64_t botStamp = bot_timestamp_sync(mBotSync, cameraStamp, wallStamp);
    return botStamp;
  }

  void operator()() {

    int64_t imageCount = 0;
    int64_t totalBytes = 0;
    int64_t prevTime = 0;

    std::cout << "Started capture thread" << std::endl;
    FlyCapture2::Error error;

    while (mIsRunning) {
      int64_t checkpointStart = bot_timestamp_now();

      // fire trigger
      if (mTrigger) {
        if (!fireSoftwareTrigger()) {
          std::cout << "error firing trigger" << std::endl;
        }
      }

      // grab image and convert to rgb
      FlyCapture2::Image rawImage;
      error = mCamera.RetrieveBuffer(&rawImage);
      if (error != FlyCapture2::PGRERROR_OK) {
        std::cout << "capture error" << std::endl;
        continue;
      }
      FlyCapture2::Image rgbImage;
      rawImage.Convert(FlyCapture2::PIXEL_FORMAT_RGB, &rgbImage);

      // convert to opencv
      cv::Mat cvImage;
      convert(rgbImage, cvImage);

      // convert to libbot image type
      bot_core::image_t msg;
      msg.utime = getImageTime(rawImage);
      convert(cvImage, msg);

      // transmit message
      mLcm->publish(mPublishChannel, &msg);

      // print frame rate
      ++imageCount;
      totalBytes += msg.data.size();
      if (prevTime == 0) prevTime = msg.utime;
      double timeDelta = (msg.utime - prevTime)/1e6;
      if (timeDelta > 5) {
        double bitRate = totalBytes/timeDelta*8/1024;
        std::string rateUnit = "Kbps";
        if (bitRate > 1024) {
          bitRate /= 1024;
          rateUnit = "Mbps";
        }
        fprintf(stdout, "%.1f Hz, %.2f %s\n", imageCount/timeDelta, bitRate,
                rateUnit.c_str());

        prevTime = msg.utime;
        imageCount = 0;
        totalBytes = 0;
      }

      // sleep if necessary
      int64_t elapsedMs = (bot_timestamp_now() - checkpointStart)/1000;
      if (mPeriodMs > 0) {
        std::this_thread::sleep_for
          (std::chrono::milliseconds(mPeriodMs - elapsedMs));
      }
    }

    std::cout << "Stopping capture..." << std::flush;
    bool good = false;
    error = mCamera.StopCapture();
    if (error == FlyCapture2::PGRERROR_OK) {
      error = mCamera.Disconnect();
      if (error == FlyCapture2::PGRERROR_OK) {
        good = true;
      }
    }
    std::cout << (good ? "done" : "failed") << std::endl;

    std::cout << "Exited capture thread" << std::endl;
  }

  bool fireSoftwareTrigger() {
    FlyCapture2::Error error;
    unsigned int regVal = 0;
    do {
      error = mCamera.ReadRegister(0x62C, &regVal);
      if (error != FlyCapture2::PGRERROR_OK) return false;
    }
    while ((regVal >> 31) != 0);
    error = mCamera.FireSoftwareTrigger();
    return (error == FlyCapture2::PGRERROR_OK);
  }

  bool streamFrames(const int iTimeMs) {
    std::cout << "Streaming frames for " << iTimeMs << " ms..." << std::flush;
    FlyCapture2::Error error;
    bool good = true;
    int numCaptured = 0;
    int64_t startTime = bot_timestamp_now();
    int64_t curTime = startTime;
    while ((curTime-startTime)/1e3 < iTimeMs) {
      if (mTrigger) {
        if (!fireSoftwareTrigger()) {
          std::cout << "error firing trigger" << std::endl;
        }
      }
      FlyCapture2::Image rawImage;
      error = mCamera.RetrieveBuffer(&rawImage);
      curTime = bot_timestamp_now();
      if (error == FlyCapture2::PGRERROR_OK) ++numCaptured;
      //else good = false;
    }
    if (numCaptured == 0) good = false;
    std::cout << (good ? "done" : "failed") << std::endl;
    std::cout << "streamed " << numCaptured << " frames" << std::endl;
    return good;
  }

  bool findCameraBySerial() {
    FlyCapture2::Error error;
    bool foundCamera = false;
    FlyCapture2::BusManager busManager;
    unsigned int numCameras;
    error = busManager.GetNumOfCameras(&numCameras);
    if (error != FlyCapture2::PGRERROR_OK) {
      std::cout << "Could not get number of cameras" << std::endl;
      return false;
    }

    for (int i = 0; i < numCameras; ++i) {
      unsigned int curSerial;
      error = busManager.GetCameraSerialNumberFromIndex(i, &curSerial);
      if (error != FlyCapture2::PGRERROR_OK) {
        std::cout << "Could not get serial number from index " <<
          i << std::endl;
        return false;
      }

      if (curSerial == mDesiredSerial) {
        error = busManager.GetCameraFromIndex(i, &mCameraUid);
        if (error != FlyCapture2::PGRERROR_OK) {
          std::cout << "Could not get camera id from index " <<
            i << std::endl;
          return false;
        }
        foundCamera = true;
        break;
      }
    }
    if (!foundCamera) {
      std::cout << "Could not find camera with serial " <<
        mDesiredSerial << std::endl;
      return false;
    }
    return true;
  }

  bool setProperty(const FlyCapture2::PropertyType iType, const bool iAbs,
                   const bool iAuto, const bool iOn, const float iValue) {
    FlyCapture2::Property prop;
    prop.type = iType;
    prop.absControl = iAbs;
    prop.onePush = false;
    prop.onOff = iOn;
    prop.autoManualMode = iAuto;
    prop.valueA = prop.valueB = 0;
    prop.absValue = iValue;
    FlyCapture2::Error error = mCamera.SetProperty(&prop);
    if (error != FlyCapture2::PGRERROR_OK) {
      std::cout << error.GetDescription() << std::endl;
      return false;
    }
    return true;
  }

  bool setProperties() {
    bool good = true;
    std::cout << "Adjusting camera settings..." << std::flush;
    good &= setProperty(FlyCapture2::BRIGHTNESS, true, true, false, 0);
    good &= setProperty(FlyCapture2::AUTO_EXPOSURE, true, false, true, 0);
    good &= setProperty(FlyCapture2::SHUTTER, true, true, false, 0);
    good &= setProperty(FlyCapture2::GAIN, true, true, false, 0);
    good &= setProperty(FlyCapture2::WHITE_BALANCE, false, true, true, 0);
    if (good) {
      FlyCapture2::Error error;
      uint32_t regVal = 0;
      error = mCamera.ReadRegister(0x12F8, &regVal);
      good = (error == FlyCapture2::PGRERROR_OK);
      if (good) {
        regVal |= (1);
        error = mCamera.WriteRegister(0x12F8, regVal);
        good = (error == FlyCapture2::PGRERROR_OK);
      }
    }
    std::cout << (good ? "done" : "failed") << std::endl;
    return good;
  }

  bool setConfig() {
    FlyCapture2::FC2Config config;
    FlyCapture2::Error error;
    bool good;
    std::cout << "Setting camera configuration..." << std::flush;
    error = mCamera.GetConfiguration(&config);
    if (error == FlyCapture2::PGRERROR_OK) {
      config.grabTimeout = 2000;
      error = mCamera.SetConfiguration(&config);
      if (error == FlyCapture2::PGRERROR_OK) {
        good = true;
      }
    }
    std::cout << (good ? "done" : "failed") << std::endl;
    return good;
  }

  bool setTriggerMode() {
    FlyCapture2::Error error;
    FlyCapture2::TriggerModeInfo triggerModeInfo;
    FlyCapture2::TriggerMode triggerMode;
    bool good = false;

    std::cout << "Setting trigger mode..." << std::flush;
    error = mCamera.GetTriggerModeInfo(&triggerModeInfo);
    if (error == FlyCapture2::PGRERROR_OK) {
      error = mCamera.GetTriggerMode(&triggerMode);
      if (error == FlyCapture2::PGRERROR_OK) {
        triggerMode.onOff = mTrigger;
        triggerMode.mode = 0;
        triggerMode.parameter = 0;
        triggerMode.source = 7;
        error = mCamera.SetTriggerMode(&triggerMode);
        if (error == FlyCapture2::PGRERROR_OK) {
          good = true;
        }
      }
    }
    if (!good) {
      std::cout << "failed" << std::endl;
      if (mTrigger) {
        std::cout << "warning: cannot use trigger mode" << std::endl;
      }
      else {
        std::cout << "warning: cannot unset trigger mode" << std::endl;
      }
      mTrigger = !mTrigger;
    }
    else {
      std::cout << "done" << std::endl;
    }

    return good;
  }

  bool stop() {
    if (mCaptureThread.joinable()) mCaptureThread.join();
    return true;
  }

};

int main(const int iArgc, const char** iArgv) {
  // create state object
  State state;

  // parse program arguments
  int maxAttempts = 3;
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(state.mPublishChannel, "c", "channel", "output channel");
  opt.add(state.mCompressionQuality, "q", "quality",
          "compression quality (100=no compression)");
  opt.add(state.mFrameRate, "f", "framerate", "frame rate (0=max)");
  opt.add(state.mRotation, "r", "rotation", "rotation (0,90,180,270)");
  opt.add(state.mDesiredSerial, "s", "serial number");
  opt.add(state.mDesiredIpAddress, "p", "ip-address", "ip address");
  opt.add(state.mTrigger, "t", "trigger", "whether to use trigger mode");
  opt.add(state.mInitSeconds, "i", "init", "number of seconds for init");
  opt.add(state.mDelaySeconds, "d", "delay", "number of seconds before init");
  opt.add(state.mEqualizeRadius, "e", "equalize",
          "radius in pixels of equalization region");
  opt.add(maxAttempts, "a", "attempts", "number of tries to start up");
  opt.parse();

  // validate rotation
  if ((state.mRotation != 0) && (state.mRotation != 90) &&
      (state.mRotation != 180) && (state.mRotation != 270)) {
    std::cout << "error: invalid rotation " << state.mRotation << std::endl;
    return -1;
  }

  // multiple attempts at startup
  bool success = false;
  for (int attempt = 0; attempt < maxAttempts; ++attempt) {
    std::cout << "startup attempt " << attempt << std::endl;
    success = state.start();
    if (success) break;
  }
  if (!success) {
    std::cout << "error: problem starting camera after " << maxAttempts <<
      " attempts" << std::endl;
  }

  if (state.mCaptureThread.joinable()) state.mCaptureThread.join();

  return 0;
}
