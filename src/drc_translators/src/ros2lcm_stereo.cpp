// Copyright 2015 Maurice Fallon

// Synchronized Stereo Translator
//
// TODO(tbd):
// rgb bgr conversion
// grey compression
// disparity/depth conversion from device

// rosparam set /compressed_listener/image_transport compressed;
// rosparam set /ros2lcm/stereo hand; rosrun translators  ros2lcm_stereo  __name:=ros2lcm_hand
//
// rosparam set /ros2lcm_head/image_transport compressed;
// rosparam set /ros2lcm/stereo head;  rosrun translators ros2lcm_stereo  __name:=ros2lcm_head
//
// rosparam set /compressed_listener/image_transport compressed
// rosrun translators  my_subscriber __name:=compressed_listener

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <opencv2/opencv.hpp>

class App
{
public:
  explicit App(ros::NodeHandle node_in);
  ~App();

private:
  lcm::LCM lcm_publish_;
  ros::NodeHandle node_;

  // Combined Stereo Image:
  bot_core::images_t images_msg_out_;
  bot_core::image_t image_a_lcm_;
  bot_core::image_t image_b_lcm_;
  void publishStereo(const sensor_msgs::ImageConstPtr& image_a_ros, const sensor_msgs::CameraInfoConstPtr& info_a_ros,
                     const sensor_msgs::ImageConstPtr& image_ros_b, const sensor_msgs::CameraInfoConstPtr& info_b_ros,
                     std::string camera_out);
  image_transport::ImageTransport it_;

  ///////////////////////////////////////////////////////////////////////////////
  void head_stereo_cb(const sensor_msgs::ImageConstPtr& image_a_ros, const sensor_msgs::CameraInfoConstPtr& info_cam_a,
                      const sensor_msgs::ImageConstPtr& image_ros_b, const sensor_msgs::CameraInfoConstPtr& info_cam_b);
  image_transport::SubscriberFilter image_a_ros_sub_, image_b_ros_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_a_ros_sub_, info_b_ros_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
      sensor_msgs::CameraInfo> sync_;

  bool do_jpeg_compress_;
  int jpeg_quality_;
  bool do_zlib_compress_;
  int depth_compress_buf_size_;
  uint8_t* depth_compress_buf_;

  void prepImage(bot_core::image_t& lcm_image, const sensor_msgs::ImageConstPtr& ros_image);
};

App::App(ros::NodeHandle node_in) :
    node_(node_in), it_(node_in), sync_(10)
{
  if (!lcm_publish_.good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  std::string image_a_string, info_a_string, image_b_string, info_b_string;
  std::string head_stereo_root = "";

  image_a_string = head_stereo_root + "/left/image_rect_color";
  info_a_string = image_a_string + "/camera_info";
  // rim_string = head_stereo_root + "/right/image_rect_color";
  image_b_string = head_stereo_root + "/right/image_rect";
  info_b_string = image_b_string + "/camera_info";
  std::cout << image_a_string << " is the image_a topic subscription [for stereo]\n";
  image_a_ros_sub_.subscribe(it_, ros::names::resolve(image_a_string), 30);
  info_a_ros_sub_.subscribe(node_, ros::names::resolve(info_a_string), 30);
  image_b_ros_sub_.subscribe(it_, ros::names::resolve(image_b_string), 30);
  info_b_ros_sub_.subscribe(node_, ros::names::resolve(info_b_string), 30);
  sync_.connectInput(image_a_ros_sub_, info_a_ros_sub_, image_b_ros_sub_, info_b_ros_sub_);
  sync_.registerCallback(boost::bind(&App::head_stereo_cb, this, _1, _2, _3, _4));

  images_msg_out_.images.push_back(image_a_lcm_);
  images_msg_out_.images.push_back(image_b_lcm_);
  images_msg_out_.image_types.push_back(0);
  images_msg_out_.image_types.push_back(2);
  // bot_core::images_t::LEFT
  // 0 left, 1 right, 2 DISPARITY, 3 maskzipped, 4 depth mm, 5 DISPARITY_ZIPPED, 6 DEPTH_MM_ZIPPED

  // allocate space for zlib compressing depth data

  depth_compress_buf_size_ = 800 * 800 * sizeof(int8_t) * 10;
  depth_compress_buf_ = (uint8_t*)malloc(depth_compress_buf_size_);
  do_jpeg_compress_ = true;
  jpeg_quality_ = 95;  // 95 is opencv default
  do_zlib_compress_ = true;
}

App::~App()
{
}

int stereo_counter = 0;
void App::head_stereo_cb(const sensor_msgs::ImageConstPtr& image_a_ros,
                         const sensor_msgs::CameraInfoConstPtr& info_a_ros,
                         const sensor_msgs::ImageConstPtr& image_b_ros,
                         const sensor_msgs::CameraInfoConstPtr& info_b_ros)
{
  int64_t current_utime = (int64_t)floor(image_a_ros->header.stamp.toNSec() / 1000);
  publishStereo(image_a_ros, info_a_ros, image_b_ros, info_b_ros, "CAMERA");

  if (stereo_counter % 30 == 0)
  {
    ROS_ERROR("HDCAM [%d]", stereo_counter);
  }
  stereo_counter++;
}

void App::publishStereo(const sensor_msgs::ImageConstPtr& image_a_ros,
                        const sensor_msgs::CameraInfoConstPtr& info_a_ros,
                        const sensor_msgs::ImageConstPtr& image_ros_b,
                        const sensor_msgs::CameraInfoConstPtr& info_b_ros, std::string camera_out)
{
  prepImage(image_a_lcm_, image_a_ros);
  prepImage(image_b_lcm_, image_ros_b);

  images_msg_out_.images[0] = image_a_lcm_;
  images_msg_out_.images[1] = image_b_lcm_;
  images_msg_out_.image_types[0] = 0;
  images_msg_out_.image_types[1] = 1;

  images_msg_out_.n_images = images_msg_out_.images.size();
  images_msg_out_.utime = (int64_t)floor(image_a_ros->header.stamp.toNSec() / 1000);
  lcm_publish_.publish("CAMERA", &images_msg_out_);
  return;
}

void App::prepImage(bot_core::image_t& lcm_image, const sensor_msgs::ImageConstPtr& ros_image)
{
  int64_t current_utime = (int64_t)floor(ros_image->header.stamp.toNSec() / 1000);
  lcm_image.utime = current_utime;
  int isize = ros_image->width * ros_image->height;
  int n_colors;

  if ((ros_image->encoding.compare("mono8") == 0)
      || ((ros_image->encoding.compare("rgb8") == 0) || (ros_image->encoding.compare("bgr8") == 0)))
  {
    if (ros_image->encoding.compare("mono8") == 0)
    {
      n_colors = 1;
    }
    else if (ros_image->encoding.compare("rgb8") == 0)
    {
      n_colors = 3;
    }
    else if (ros_image->encoding.compare("bgr8") == 0)
    {
      n_colors = 3;
    }
    else
    {
      std::cout << "Encoding [" << ros_image->encoding << "] not supported\n";
      exit(-1);
    }

    void* bytes = const_cast<void*>(static_cast<const void*>(ros_image->data.data()));
    cv::Mat mat;
    if (n_colors == 1)
    {
      mat = cv::Mat(ros_image->height, ros_image->width, CV_8UC1, bytes, n_colors * ros_image->width);
    }
    else if (n_colors == 3)
    {
      mat = cv::Mat(ros_image->height, ros_image->width, CV_8UC3, bytes, n_colors * ros_image->width);
    }
    else
    {
      std::cout << "Number of colors [" << n_colors << "] not supported\n";
      exit(-1);
    }

    if (do_jpeg_compress_)
    {
      if (ros_image->encoding.compare("rgb8") == 0)  // non intuative color flip needed here
        cv::cvtColor(mat, mat, CV_BGR2RGB);

      std::vector<int> params;
      params.push_back(cv::IMWRITE_JPEG_QUALITY);
      params.push_back(jpeg_quality_);
      cv::imencode(".jpg", mat, lcm_image.data, params);
      lcm_image.size = lcm_image.data.size();
      lcm_image.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
    }
    else
    {
      if (ros_image->encoding.compare("bgr8") == 0)
        cv::cvtColor(mat, mat, CV_BGR2RGB);

      lcm_image.data.resize(n_colors * isize);
      memcpy(&lcm_image.data[0], mat.data, n_colors * isize);
      lcm_image.size = n_colors * isize;
      if (n_colors == 1)
      {
        lcm_image.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY;
      }
      else if (n_colors == 3)
      {
        lcm_image.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
      }
      else
      {
        std::cout << "Number of colors [" << n_colors << "] not supported\n";
        exit(-1);
      }
    }
  }
  else if (1 == 2)
  {
    std::cout << ros_image->encoding << " | encoded not fully working - FIXME\n";
    exit(-1);
    return;

    n_colors = 2;  // 2 bytes per pixel

    /*
     int uncompressed_size = isize;
     // Insert proper compression here if needed:
     unsigned long compressed_size = depth_compress_buf_size_;
     compress2( depth_compress_buf_, &compressed_size, (const Bytef*) imageDataP, uncompressed_size,
     Z_BEST_SPEED);
     lcm_image.data.resize(compressed_size);
     */
    unsigned long zlib_compressed_size = 1000;  // fake compressed size
    lcm_image.data.resize(zlib_compressed_size);
    lcm_image.size = zlib_compressed_size;
    // images_msg_out_.image_types[1] = 5;// bot_core::images_t::DISPARITY_ZIPPED );
  }
  else
  {
    std::cout << ros_image->encoding << " | encoded not fully working - FIXME\n";
    exit(-1);
    return;

    n_colors = 2;  // 2 bytes per pixel

    lcm_image.data.resize(2 * isize);
    lcm_image.size = 2 * isize;
    // images_msg_out_.image_types[1] = 2;// bot_core::images_t::DISPARITY );
  }

  lcm_image.width = ros_image->width;
  lcm_image.height = ros_image->height;
  lcm_image.nmetadata = 0;
  lcm_image.row_stride = n_colors * ros_image->width;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros2lcm_stereo");
  std::string which_camera = "head";

  ros::NodeHandle nh;
  // Disabled, originally used for jpeg-in-jpeg out:
  /*
   std::string  which_transport;
   which_transport = "/ros2lcm_head";
   std::string transport;
   if (nh.getParam(std::string( which_transport + "/image_transport"), transport)) {
   std::cout << "transport is " << transport << "\n";
   }
   ROS_ERROR("Stereo Camera Translator: [%s] [%s]", which_camera.c_str() , transport.c_str());
   */
  ROS_ERROR("Stereo Camera Translator: [%s]", which_camera.c_str());

  new App(nh);
  std::cout << "ros2lcm translator ready\n";
  ros::spin();
  return 0;
}
