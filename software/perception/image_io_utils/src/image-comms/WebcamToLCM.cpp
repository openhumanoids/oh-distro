// Copyright 2016 Wolfgang Merkt

#include <lcm/lcm-cpp.hpp>
#include <bot_core/timestamp.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ConciseArgs>

#include <algorithm>
#include <iostream>
#include <vector>

#include "lcmtypes/bot_core/image_t.hpp"
#include "lcmtypes/bot_core/images_t.hpp"

int64_t bot_timestamp_now() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

int main(int argc, char **argv) {
  bool compress_images = true;
  bool simulate_multisense_camera_left = false;
  int jpeg_quality = 90;

  ConciseArgs opt(argc, argv);
  opt.add(compress_images, "nc", "dont_compress_images",
          "Turn off JPEG compression of images");
  opt.add(simulate_multisense_camera_left, "m", "simulate_multisense",
          "send CAMERA images_t");
  opt.add(jpeg_quality, "j", "jpeg_quality", "jpeg quality (1-100)");
  opt.parse();

  std::cout << "Compress JPEG: " << std::to_string(compress_images)
            << " (quality: " << jpeg_quality << ")" << std::endl;
  std::cout << "Simulate Multisense CAMERA images_t output: "
            << std::to_string(simulate_multisense_camera_left) << std::endl;

  lcm::LCM lcm_handle;

  if (!lcm_handle.good()) std::cerr << "ERROR: lcm is not good()" << std::endl;

  cv::VideoCapture cap;
  if (!cap.open(0)) {
    std::cerr << "Could not open webcam, exiting" << std::endl;
    return 1;
  }
  uint width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  uint height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

  cv::Mat frame;
  std::vector<int> params;
  params.push_back(cv::IMWRITE_JPEG_QUALITY);
  params.push_back(jpeg_quality);

  bot_core::image_t lcm_img;
  lcm_img.width = width;
  lcm_img.height = height;
  lcm_img.nmetadata = 0;
  int n_colors = 3;
  lcm_img.row_stride = n_colors * width;

  bot_core::images_t lcm_imgs;
  lcm_imgs.image_types.push_back(0);  // 0 = left
  lcm_imgs.images.push_back(lcm_img);
  lcm_imgs.n_images = lcm_imgs.images.size();

  std::cout << "Image resolution: " << width << "x" << height << std::endl;

  for (;;) {
    cap >> frame;
    if (frame.empty()) break;

    // cv::imshow("Webcam Video", frame);
    lcm_img.utime = bot_timestamp_now();

    if (!compress_images) {
      lcm_img.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
      lcm_img.size = frame.step * frame.rows;
      lcm_img.data.resize(frame.step * frame.rows);
      std::copy(frame.data, frame.data + frame.step * frame.rows,
                lcm_img.data.begin());
    } else {
      cv::imencode(".jpg", frame, lcm_img.data, params);
      lcm_img.size = lcm_img.data.size();
      lcm_img.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
    }

    lcm_imgs.images[0] = lcm_img;
    lcm_imgs.utime = lcm_img.utime;

    lcm_handle.publish("WEBCAM", &lcm_img);

    if (simulate_multisense_camera_left)
      lcm_handle.publish("CAMERA", &lcm_imgs);

    if (cv::waitKey(1) == 27) break;  // press ESC to stop
  }

  cap.release();
  return 0;
}
