#include <iostream>
#include <thread>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <bot_core/timestamp.h>
#include <jpeg-utils/jpeg-utils.h>
#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>
#include <flycapture/FlyCapture2.h>
#include <lcmtypes/bot_core/image_t.hpp>
 
int main(const int iArgc, const char** iArgv) {

  // parse arguments
  std::string channel = "DUMMY_CAMERA_CHANNEL";
  int compressionQuality = 100;
  int frameRate = 0;
  int rotation = 0;
  int64_t desiredSerial = -1;
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(channel, "c", "channel", "output channel");
  opt.add(compressionQuality, "q", "quality",
          "compression quality (100=no compression)");
  opt.add(frameRate, "f", "framerate", "frame rate (0=max)");
  opt.add(rotation, "r", "rotation", "rotation (0,90,180,270)");
  opt.add(desiredSerial, "s", "serial number");
  opt.parse();

  if ((rotation != 0) && (rotation != 90) &&
      (rotation != 180) && (rotation != 270)) {
    std::cout << "error: invalid rotation " << rotation << std::endl;
    return -1;
  }

  bool shouldCompress = (compressionQuality < 100);
  int periodMs = (frameRate == 0 ? 0 : 1000/frameRate);
  lcm::LCM lcm;

  // determine which camera id to connect to
  FlyCapture2::Error error;
  FlyCapture2::PGRGuid cameraUid;
  bool foundCamera = false;
  if (desiredSerial >= 0) {
    std::cout << "About to create FlyCapture2 Object\n";
    FlyCapture2::BusManager busManager;
    std::cout << "... finished creating\n";
    unsigned int numCameras;
    error = busManager.GetNumOfCameras(&numCameras);
    if (error != FlyCapture2::PGRERROR_OK) {
      std::cout << "Could not get number of cameras" << std::endl;
      return -1;
    }

    for (int i = 0; i < numCameras; ++i) {
      unsigned int curSerial;
      error = busManager.GetCameraSerialNumberFromIndex(i, &curSerial);
      if (error != FlyCapture2::PGRERROR_OK) {
        std::cout << "Could not get serial number from index " <<
          i << std::endl;
        return -1;
      }

      if (curSerial == desiredSerial) {
        error = busManager.GetCameraFromIndex(i, &cameraUid);
        if (error != FlyCapture2::PGRERROR_OK) {
          std::cout << "Could not get camera id from index " <<
            i << std::endl;
          return -1;
        }
        foundCamera = true;
        break;
      }
    }
    if (!foundCamera) {
      std::cout << "Could not find camera with serial " <<
        desiredSerial << std::endl;
      return -1;
    }
  }

  FlyCapture2::Camera camera;
  FlyCapture2::CameraInfo camInfo;
 
  // connect to camera
  if (desiredSerial >= 0) {
    error = camera.Connect(&cameraUid);
  }
  else {
    error = camera.Connect(NULL);
  }
  if (error != FlyCapture2::PGRERROR_OK) {
    std::cout << "Failed to connect to camera" << std::endl;     
    return -1;
  }

  // stop capture if in bad state
  error = camera.StopCapture();
  if (error != FlyCapture2::PGRERROR_OK) {}

  // get camera info
  error = camera.GetCameraInfo(&camInfo);
  if (error != FlyCapture2::PGRERROR_OK) {
    std::cout << "Failed to get camera info from camera" << std::endl;     
    return -1;
  }
  std::cout << "Camera information: "
            << camInfo.vendorName << " "
            << camInfo.modelName << " " 
            << camInfo.serialNumber << std::endl;

  // set trigger mode
  bool usingTrigger = false;
  FlyCapture2::TriggerModeInfo triggerModeInfo;
  error = camera.GetTriggerModeInfo(&triggerModeInfo);
  if (error == FlyCapture2::PGRERROR_OK) {
    FlyCapture2::TriggerMode triggerMode;
    error = camera.GetTriggerMode(&triggerMode);
    if (error == FlyCapture2::PGRERROR_OK) {
      triggerMode.onOff = true;
      triggerMode.mode = 0;
      triggerMode.parameter = 0;
      triggerMode.source = 7;
      error = camera.SetTriggerMode(&triggerMode);
      if (error == FlyCapture2::PGRERROR_OK) {
        usingTrigger = true;
      }
    }
  }
  if (!usingTrigger) {
    std::cout << "warning: cannot use trigger mode; " <<
      "cpu utilization may be high" << std::endl;
  }

  error = camera.StartCapture();
  if (error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
    std::cout << "Bandwidth exceeded" << std::endl;     
    return -1;
  }
  else if (error != FlyCapture2::PGRERROR_OK) {
    std::cout << "Failed to start image capture" << std::endl;     
    return -1;
  }

  int64_t imageCount = 0;
  int64_t totalBytes = 0;
  int64_t prevTime = 0;

  while (true) {
    int64_t checkpointStart = bot_timestamp_now();

    // fire trigger
    if (usingTrigger) {
      error = camera.FireSoftwareTrigger();
      if (error != FlyCapture2::PGRERROR_OK) {
        std::cout << "error firing trigger" << std::endl;
      }
    }

    // grab image
    FlyCapture2::Image rawImage;
    error = camera.RetrieveBuffer(&rawImage);
    if (error != FlyCapture2::PGRERROR_OK) {
      std::cout << "capture error" << std::endl;
      continue;
    }

    // TODO: this can be grabbed from image metadata via GetTimeStamp()
    int64_t imageTime = bot_timestamp_now();

    // convert to rgb
    FlyCapture2::Image rgbImage;
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_RGB, &rgbImage);

    // convert to opencv
    cv::Mat cvImage(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3,
                    rgbImage.GetData(), rgbImage.GetStride());
       
    // rotate
    switch (rotation) {
    case 0:
      break;
    case 90:
      cv::transpose(cvImage, cvImage);
      cv::flip(cvImage, cvImage, 1);
      break;
    case 180:
      cv::flip(cvImage, cvImage, -1);
      break;
    case 270:
      cv::transpose(cvImage, cvImage);
      cv::flip(cvImage, cvImage, 0);
      break;
    default:
      std::cout << "error: cannot rotate by " << rotation << std::endl;
      break;
    }

    // convert to libbot image type
    bot_core::image_t msg;
    msg.utime = imageTime;
    msg.width = cvImage.cols;
    msg.height = cvImage.rows;
    msg.row_stride = cvImage.step;
    msg.nmetadata = 0;

    // compress if necessary
    uint8_t* data = cvImage.data;
    msg.size = msg.height*msg.row_stride;
    if (shouldCompress) {
      std::vector<uint8_t> dest(msg.height*msg.row_stride);
      jpeg_compress_8u_rgb(data, msg.width, msg.height, msg.row_stride,
                           dest.data(), &msg.size, compressionQuality);
      msg.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
      msg.data.resize(msg.size);
      std::copy(dest.data(), dest.data()+msg.size, msg.data.begin());
    }

    // otherwise just set raw bytes
    else {
      msg.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
      msg.data.resize(msg.size);
      std::copy(data, data + msg.size, msg.data.begin());
    }

    // transmit message
    lcm.publish(channel, &msg);

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
    if (periodMs > 0) {
      std::this_thread::sleep_for
        (std::chrono::milliseconds(periodMs - elapsedMs));
    }
    std::cout << bot_timestamp_now() << " tic\n";
  }

  // clean up
  if (usingTrigger) {
    FlyCapture2::TriggerMode triggerMode;
    error = camera.GetTriggerMode(&triggerMode);
    if (error == FlyCapture2::PGRERROR_OK) {
      triggerMode.onOff = false;
      error = camera.SetTriggerMode(&triggerMode);
    }
    if (error != FlyCapture2::PGRERROR_OK) {
      std::cout << "warning: cannot unset trigger mode" << std::endl;
    }
  }
  error = camera.StopCapture();
  if (error != FlyCapture2::PGRERROR_OK) {}
  camera.Disconnect();
        
  return 0;
}
