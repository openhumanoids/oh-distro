// Copyright 2015 Maurice Fallon, Wolfgang Merkt

// ros2lcm translator for images

// ### Boost
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

// ### ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

// ### Standard includes
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <algorithm>

// ### LCM
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

// ### OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class App
{
public:
  explicit App(ros::NodeHandle node_);
  ~App();

private:
  lcm::LCM lcmPublish_;
  ros::NodeHandle node_;

  ros::Subscriber headLeftImageSub_;

  void headLeftImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void publishImage(const sensor_msgs::ImageConstPtr& msg, std::string channel);

  bool flip_rgb;
};

App::App(ros::NodeHandle node_) :
    flip_rgb(false)
{
  ROS_INFO("Initializing Translator");
  if (!lcmPublish_.good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  std::string cameraTopic;
  ros::NodeHandle nh_("~");
  nh_.getParam("camera_topic", cameraTopic);
  nh_.getParam("flip_rgb", flip_rgb);
  std::cout << "Subscribing to " << cameraTopic << std::endl;
  std::cout << "flip_rgb: " << static_cast<int>(flip_rgb) << std::endl;
  headLeftImageSub_ = node_.subscribe(cameraTopic, 1, &App::headLeftImageCallback, this);
}

App::~App()
{
}

int head_l_image_counter = 0;
void App::headLeftImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (head_l_image_counter % 80 == 0)
  {
    ROS_ERROR("CAMLT [%d]", head_l_image_counter);
  }
  head_l_image_counter++;
  publishImage(msg, "CAMERA_LEFT");
}

void App::publishImage(const sensor_msgs::ImageConstPtr& msg, std::string channel)
{
  int64_t current_utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000);
  int n_colors = 3;
  int isize = msg->data.size();
  bot_core::image_t lcm_img;
  lcm_img.utime = current_utime;
  lcm_img.width = msg->width;
  lcm_img.height = msg->height;
  lcm_img.nmetadata = 0;
  lcm_img.row_stride = n_colors * msg->width;

  // TODO(tbd): reallocate to speed?
  void* bytes = const_cast<void*>(static_cast<const void*>(msg->data.data()));
  cv::Mat mat(msg->height, msg->width, CV_8UC3, bytes, lcm_img.row_stride);

  bool compress_images = true;
  if (!compress_images)
  {
    lcm_img.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
    lcm_img.size = isize;
    cv::cvtColor(mat, mat, CV_RGB2BGR);  // TODO(tbd): expose this as a param
    // cv::cvtColor(mat, mat, CV_BGR2RGB);
    lcm_img.data.resize(mat.step * mat.rows);
    std::copy(mat.data, mat.data + mat.step * mat.rows, lcm_img.data.begin());
  }
  else
  {
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(90);

    if (flip_rgb)
      cv::cvtColor(mat, mat, CV_BGR2RGB);

    cv::imencode(".jpg", mat, lcm_img.data, params);
    lcm_img.size = lcm_img.data.size();
    lcm_img.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
  }

  lcmPublish_.publish(channel.c_str(), &lcm_img);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros2lcm_camera");
  ros::NodeHandle nh;
  new App(nh);
  ROS_ERROR("ROS2LCM Camera Translator Ready");
  ros::spin();
  return 0;
}
