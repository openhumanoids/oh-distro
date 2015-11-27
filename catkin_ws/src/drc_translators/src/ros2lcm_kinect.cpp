// Copyright 2015 Maurice Fallon

// ### Standard includes
#include <algorithm>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <lcm/lcm-cpp.hpp>
#include <zlib.h>

#include "lcmtypes/kinect.hpp"
#include "lcmtypes/bot_core.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

lcm::LCM* lcmThe(new lcm::LCM);

// Maximum size of reserved buffer. This will support and images smaller than this also:
int local_img_buffer_size_ = 640 * 480 * sizeof(int16_t) * 4;
uint8_t* local_img_buffer_ = new uint8_t[local_img_buffer_size_];  // x4 was used for zlib in kinect_lcm
// is x10 necessary for  jpeg? thats waht kinect_lcm assumed

void callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth)
{
  /*
   bot_core::image_t lcm_img;
   lcm_img.utime =0;//msg->image.timestamp;
   lcm_img.width =rgb->width;
   lcm_img.height =rgb->height;
   lcm_img.nmetadata =0;
   int n_colors = 3;
   lcm_img.row_stride=n_colors*rgb->width;
   //    if (msg->image.image_data_format == kinect::image_msg_t::VIDEO_RGB){
   lcm_img.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
   lcm_img.size = rgb->data.size();
   lcm_img.data = rgb->data;
   lcmThe->publish("KINECT_RGB", &lcm_img);
   */

  kinect::image_msg_t lcm_rgb;
  lcm_rgb.timestamp = (int64_t)rgb->header.stamp.toNSec() / 1000;  // from nsec to usec
  lcm_rgb.width = rgb->width;
  lcm_rgb.height = rgb->height;
  // lcm_rgb.nmetadata =0;
  int n_colors = 3;
  int isize = rgb->data.size();
  bool compress_images = true;
  if (!compress_images)
  {
    lcm_rgb.image_data_format = kinect::image_msg_t::VIDEO_RGB;  // bot_core::image_t::PIXEL_FORMAT_RGB;
    lcm_rgb.image_data_nbytes = isize;
    // cv::cvtColor(mat, mat, CV_BGR2RGB);
    // lcm_rgb.image_data.resize(mat.step*mat.rows);
    // std::copy(mat.data, mat.data + mat.step*mat.rows,
    //            lcm_rgb.image_data.begin());
    lcm_rgb.image_data = rgb->data;
  }
  else
  {
    // TODO(tbd): reallocate to speed?
    void* bytes = const_cast<void*>(static_cast<const void*>(rgb->data.data()));
    cv::Mat mat(rgb->height, rgb->width, CV_8UC3, bytes, n_colors * rgb->width);
    //cv::cvtColor(mat, mat, CV_BGR2RGB);

    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(90);

    cv::imencode(".jpg", mat, lcm_rgb.image_data, params);
    lcm_rgb.image_data_nbytes = lcm_rgb.image_data.size();
    lcm_rgb.image_data_format = kinect::image_msg_t::VIDEO_RGB_JPEG;
  }
  // lcmThe->publish("KINECT_RGBX", &lcm_rgb);

  kinect::depth_msg_t lcm_depth;
  lcm_depth.timestamp = (int64_t)depth->header.stamp.toNSec() / 1000;  // from nsec to usec
  lcm_depth.width = depth->width;
  lcm_depth.height = depth->height;
  lcm_depth.depth_data_format = kinect::depth_msg_t::DEPTH_MM;
  lcm_depth.uncompressed_size = 480 * 640 * 2;  // msg->depth.uncompressed_size; // not properly filled in

  if (1 == 0)
  {
    lcm_depth.depth_data_nbytes = depth->data.size();
    lcm_depth.depth_data = depth->data;
    lcm_depth.compression = 0;  // depth.compression;
    lcm_depth.compression = kinect::depth_msg_t::COMPRESSION_NONE;
  }
  else
  {
    int uncompressed_size = 480 * 640 * 2;
    unsigned long compressed_size = local_img_buffer_size_;
    compress2(local_img_buffer_, &compressed_size, (const Bytef*)depth->data.data(), uncompressed_size, Z_BEST_SPEED);
    lcm_depth.compression = kinect::depth_msg_t::COMPRESSION_ZLIB;

    lcm_depth.depth_data.resize(compressed_size);
    std::copy(local_img_buffer_, local_img_buffer_ + compressed_size, lcm_depth.depth_data.begin());
    lcm_depth.depth_data_nbytes = compressed_size;
  }
  // lcmThe->publish("KINECT_DEPTH_ONLY", &lcm_depth);

  kinect::frame_msg_t out;
  out.timestamp = (int64_t)rgb->header.stamp.toNSec() / 1000;  // from nsec to usec
  out.image = lcm_rgb;
  out.depth = lcm_depth;
  lcmThe->publish("KINECT_FRAME", &out);
}

int main(int argc, char** argv)
{
  if (!lcmThe->good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  ros::init(argc, argv, "kinect_node");

  ros::NodeHandle nh;
  // rgb: image_color, image_rect_color
  // depth: 32FC1: image, image_rect and 16UC1: image_raw, image_rect_raw

  message_filters::Subscriber<sensor_msgs::Image> image1_sub(nh, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> image2_sub(nh, "/camera/depth/image_raw", 1);
  //message_filters::Subscriber<sensor_msgs::Image> image1_sub(nh, "/camera/rgb/image_rect_color", 1);
  //message_filters::Subscriber<sensor_msgs::Image> image2_sub(nh, "/camera/depth/image_rect_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
