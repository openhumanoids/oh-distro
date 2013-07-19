#include <iostream>
#include <thread>
#include <chrono>

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
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(channel, "c", "channel", "output channel");
  opt.add(compressionQuality, "q", "compression quality");
  opt.add(frameRate, "f", "frame rate (0=max)");
  opt.parse();

  bool shouldCompress = (compressionQuality < 100);
  int periodMs = (frameRate == 0 ? 0 : 1000/frameRate);
  lcm::LCM lcm;

  FlyCapture2::Error error;
  FlyCapture2::Camera camera;
  FlyCapture2::CameraInfo camInfo;
 
  // connect to camera
  error = camera.Connect(0);
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
  int64_t checkpointStart = bot_timestamp_now();

  while (true) {

    // grab image
    FlyCapture2::Image rawImage;
    error = camera.RetrieveBuffer(&rawImage);
    if (error != FlyCapture2::PGRERROR_OK) {
      std::cout << "capture error" << std::endl;
      continue;
    }

    // convert to rgb
    FlyCapture2::Image rgbImage;
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_RGB, &rgbImage);
       
    // convert to libbot image type
    bot_core::image_t msg;
    msg.utime = bot_timestamp_now();
    msg.width = rgbImage.GetCols();
    msg.height = rgbImage.GetRows();
    msg.row_stride = rgbImage.GetStride();
    msg.nmetadata = 0;

    // compress if necessary
    uint8_t* data = rgbImage.GetData();
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
    checkpointStart = bot_timestamp_now();
  }

  // clean up
  error = camera.StopCapture();
  if (error != FlyCapture2::PGRERROR_OK) {}
  camera.Disconnect();
	
  return 0;
}
