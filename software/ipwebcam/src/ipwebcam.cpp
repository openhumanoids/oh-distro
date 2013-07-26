#include <iostream>
#include <thread>
#include <chrono>

#include <bot_core/timestamp.h>
#include <jpeg-utils/jpeg-utils.h>
#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>
#include <opencv2/opencv.hpp>
#include <lcmtypes/bot_core/image_t.hpp>

int main(const int iArgc, const char** iArgv) {

  // parse arguments
  std::string channel = "DUMMY_CAMERA_CHANNEL";
  int compressionQuality = 100;
  int frameRate = 0;
  std::string url = "http://admin:@10.0.0.100/video2.mjpg";
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(channel, "c", "channel", "output channel");
  opt.add(url, "u", "url", "camera url");
  opt.add(compressionQuality, "q", "compression quality");
  opt.add(frameRate, "f", "frame rate (0=max)");
  opt.parse();

  // TODO: can modify compression and grab stream directly

  bool shouldCompress = (compressionQuality < 100);
  int periodMs = (frameRate == 0 ? 0 : 1000/frameRate);
  lcm::LCM lcm;

  // start capture
  cv::Mat image, rgbImage;
  cv::VideoCapture videoCapture(url);
  if (!videoCapture.isOpened()) {
    std::cout << "error: cannot open stream " << url << std::endl;
    return -1;
  }
  else {
    std::cout << "capturing from " << url << std::endl;
  }

  int64_t imageCount = 0;
  int64_t totalBytes = 0;
  int64_t prevTime = 0;

  while (true) {
    int64_t checkpointStart = bot_timestamp_now();

    // grab image
    if (!videoCapture.read(image)) {
      std::cout << "capture error" << std::endl;
      continue;
    }

    // convert to rgb
    cv::cvtColor(image, rgbImage, CV_BGR2RGB);

    // convert to libbot image type
    bot_core::image_t msg;
    msg.utime = bot_timestamp_now();
    msg.width = rgbImage.cols;
    msg.height = rgbImage.rows;
    msg.row_stride = rgbImage.step;
    msg.nmetadata = 0;

    // compress if necessary
    uint8_t* data = rgbImage.data;
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
  }

  // clean up
  videoCapture.release();
	
  return 0;
}
