
#include <multisense_ros/camera.h>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <LibSensorPodCommunications/CamGetConfigMessage.h>
#include <LibSensorPodCommunications/CamStartStreamMessage.h>
#include <LibSensorPodCommunications/CamStartStreamAckMessage.h>
#include <LibSensorPodCommunications/CamStopStreamMessage.h>
#include <LibSensorPodCommunications/CamStopStreamAckMessage.h>

#include <LibSensorPodCommunications/CamStartImageStreamMessage.h>
#include <LibSensorPodCommunications/CamStartImageStreamAckMessage.h>
#include <LibSensorPodCommunications/CamStopImageStreamMessage.h>
#include <LibSensorPodCommunications/CamStopImageStreamAckMessage.h>
#include <LibSensorPodCommunications/CamControlMessage.h>
#include <LibSensorPodCommunications/CamControlAckMessage.h>

#include <opencv2/opencv.hpp>
#include <arpa/inet.h>


namespace multisense_ros
{

Camera::Camera(multisense_driver::MultisenseDriver* driver) :
  driver_(driver),
  stereo_nh_("stereo"),
  diagnostics_nh_(stereo_nh_, "diagnostics"),
  left_nh_(stereo_nh_, "left"),
  right_nh_(stereo_nh_, "right"),
  left_transport_(left_nh_),
  right_transport_(right_nh_),
  depth_transport_(stereo_nh_),
  got_cam_info_(false),
  depth_image_caller_(1, 5),
  point_cloud_caller_(1, 5),
  images_caller_(1, 5),
  depth_diagnostics_(ros::NodeHandle(), "stereo/depth_diagnostics", 0.0),
  image_diagnostics_(ros::NodeHandle(), "stereo/image_diagnostics", 0.0),
  config_diagnostics_(ros::NodeHandle(), "stereo/config_diagnostics", 0.0),
  reconfigure_server_(stereo_nh_)
{
  // get frame_id

  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  stereo_nh_.param("frame_id", frame_id_, std::string("/left_camera_optical_frame"));

  point_cloud_pub_ = stereo_nh_.advertise<sensor_msgs::PointCloud2>("points2", 5,
                                                                    boost::bind(&Camera::connectDepthCB, this),
                                                                    boost::bind(&Camera::disconnectDepthCB, this));

  left_cam_pub_ = left_transport_.advertiseCamera("image_rect", 5, boost::bind(&Camera::connectImageCB, this),
                                                                   boost::bind(&Camera::disconnectImageCB, this));

  right_cam_pub_ = right_transport_.advertiseCamera("image_rect", 5, boost::bind(&Camera::connectImageCB, this),
                                                                     boost::bind(&Camera::disconnectImageCB, this));

  depth_cam_pub_ = depth_transport_.advertiseCamera("depth", 5, boost::bind(&Camera::connectDepthCB, this),
                                                                boost::bind(&Camera::disconnectDepthCB, this));

  command_cam_get_config_.reset( new Command<CamGetConfigMessage, CamConfigMessage>(CamGetConfigMessage(), driver_,
                                                                                    boost::bind(&Camera::processCamConfig, this, _1)));
  command_cam_get_config_->run();

  config_diagnostics_.publish(SensorStatus::STARTING);
  stopDepthStream();
  stopImageStream();

  // subscribe to driver callbacks
  cam_data_sub_ = driver_->subscribe<CamDataMessage>(boost::bind(&Camera::processCamData, this, _1));
  cam_image_data_sub_ = driver_->subscribe<CamImageDataMessage>(boost::bind(&Camera::processCamImageData, this, _1));

  // Set up dynamic reconfigure
  reconfigure_server_.setCallback(boost::bind(&Camera::configureCB, this, _1, _2));
};



void Camera::connectDepthCB()
{
  ROS_DEBUG("Topic connection on depth created");
  if (depth_cam_pub_.getNumSubscribers() > 0 || point_cloud_pub_.getNumSubscribers() > 0)
  {
    ROS_INFO("Starting to stream depth data");
    command_cam_data_.reset( new Command<CamStartStreamMessage,
                                         CamStartStreamAckMessage>(CamStartStreamMessage(), driver_,
                                                                   boost::bind(&Camera::processCamStartStreamAck, this, _1)));
    command_cam_data_->run();

    // Diagnostics
    depth_diagnostics_.publish(SensorStatus::STARTING);
  }
}


void Camera::stopDepthStream()
{
  ROS_INFO("Stopping to stream depth data");
  command_cam_data_.reset( new Command<CamStopStreamMessage,
                                       CamStopStreamAckMessage>(CamStopStreamMessage(), driver_,
                                                                boost::bind(&Camera::processCamStopStreamAck, this, _1)));
  command_cam_data_->run();

  // Diagnostics
  depth_diagnostics_.publish(SensorStatus::STOPPING);
}


void Camera::disconnectDepthCB()
{
  ROS_DEBUG("Topic connection on depth destroyed");
  if (depth_cam_pub_.getNumSubscribers() == 0 && point_cloud_pub_.getNumSubscribers() == 0)
    stopDepthStream();
}


void Camera::connectImageCB()
{
  ROS_DEBUG("Topic connection on images created");
  if (left_cam_pub_.getNumSubscribers() > 0 || right_cam_pub_.getNumSubscribers() > 0)
  {
    ROS_INFO("Starting to stream image data");
    CamStartImageStreamMessage msg;
    msg.sendRectifiedImages = true;   // Always ask for rectified images. ROS driver doesn't support the raw interface
    command_cam_image_data_.reset( new Command<CamStartImageStreamMessage,
                                               CamStartImageStreamAckMessage>(msg, driver_,
                                                                              boost::bind(&Camera::processCamStartImageStreamAck, this, _1)));
    command_cam_image_data_->run();

    // Diagnostics
    image_diagnostics_.publish(SensorStatus::STARTING);
  }
}


void Camera::stopImageStream()
{
  ROS_INFO("Stopping to stream image data");
  command_cam_image_data_.reset( new Command<CamStopImageStreamMessage,
                                             CamStopImageStreamAckMessage>(CamStopImageStreamMessage(), driver_,
                                                                           boost::bind(&Camera::processCamStopImageStreamAck, this, _1)));
  command_cam_image_data_->run();

  // Diagnostics
  image_diagnostics_.publish(SensorStatus::STOPPING);
}


void Camera::disconnectImageCB()
{
  ROS_DEBUG("Topic connection on images destroyed");
  if (left_cam_pub_.getNumSubscribers() == 0 && right_cam_pub_.getNumSubscribers() == 0)
    stopImageStream();
}

void Camera::processCamConfig(const boost::shared_ptr<const CamConfigMessage>& msg)
{
  ROS_DEBUG("Got Camera info message");

  left_cam_info_.width = msg->width;
  left_cam_info_.height = msg->height;

  right_cam_info_.width = msg->width;
  right_cam_info_.height = msg->height;

  left_cam_info_.P[0] = msg->fx;   left_cam_info_.P[1] =     0.0;   left_cam_info_.P[2] = msg->width/2.0;   left_cam_info_.P[3] = 0.0;
  left_cam_info_.P[4] =     0.0;   left_cam_info_.P[5] = msg->fy;   left_cam_info_.P[6] = msg->height/2.0;  left_cam_info_.P[7] = 0.0;
  left_cam_info_.P[8] =     0.0;   left_cam_info_.P[9] =     0.0;   left_cam_info_.P[10]= 1.0;  left_cam_info_.P[11]= 0.0;

  right_cam_info_.P[0] = msg->fx;   right_cam_info_.P[1] =     0.0;   right_cam_info_.P[2] = msg->width/2.0;   right_cam_info_.P[3] = msg->tx * msg->fx;
  right_cam_info_.P[4] =     0.0;   right_cam_info_.P[5] = msg->fy;   right_cam_info_.P[6] = msg->height/2.0;  right_cam_info_.P[7] = 0.0;
  right_cam_info_.P[8] =     0.0;   right_cam_info_.P[9] =     0.0;   right_cam_info_.P[10]= 1.0;  right_cam_info_.P[11]= 0.0;

  got_cam_info_ = true;

  config_diagnostics_.publish(SensorStatus::RUNNING);
}

void Camera::processCamData(const boost::shared_ptr<const CamDataMessage>& msg)
{
  if (!got_cam_info_)
    return;

  if (depth_cam_pub_.getNumSubscribers() > 0)
    depth_image_caller_.addFunction(boost::bind(&Camera::processDepthImage, this, msg));
  if (point_cloud_pub_.getNumSubscribers() > 0)
    point_cloud_caller_.addFunction(boost::bind(&Camera::processPointCloud, this, msg));

  depth_diagnostics_.countStream();
}

void Camera::processDepthImage(const boost::shared_ptr<const CamDataMessage>& msg)
{
  cv::Mat_<uint16_t> disparity(msg->height, msg->width, msg->disparityImage);

  depth_image_.header.frame_id = frame_id_;
  depth_image_.header.stamp = convertTime(msg->timeStamp.getCurrentTime());

  depth_image_.height = msg->height;
  depth_image_.width  = msg->width;
  depth_image_.encoding = "32FC1";
  depth_image_.is_bigendian = (htonl(1) == 1);
  depth_image_.step = msg->width*4;
  depth_image_.data.resize(msg->height*msg->width*4);

  cv::Mat_<float> depth(msg->height, msg->width, (float*) &(depth_image_.data[0]));

  // Depth = focal_length*baseline/disparity
  // (Raw disparity message is in 16ths of pixels)
  double scale = right_cam_info_.P[3]*16.0*right_cam_info_.P[0];
  cv::divide(scale, disparity, depth);

  // Mark all 0 disparity points as NaNs
  const float bad_point = std::numeric_limits<float>::quiet_NaN();
  for (int v=0; v < msg->height; v++)
    for (int u=0; u < msg->width; u++)
      if (disparity(v,u) == 0)
        depth(v,u) = bad_point;

  left_cam_info_.header = depth_image_.header;
  depth_cam_pub_.publish(depth_image_, left_cam_info_);
}

void Camera::processPointCloud(const boost::shared_ptr<const CamDataMessage>& msg)
{
  right_cam_info_.header.frame_id = frame_id_;
  left_cam_info_.header.frame_id = frame_id_;
  point_cloud_converter_.toPointCloud2(left_cam_info_, right_cam_info_, *msg, point_cloud_);
  point_cloud_.header.frame_id = frame_id_;
  point_cloud_.header.stamp = convertTime(msg->timeStamp.getCurrentTime());
  point_cloud_pub_.publish(point_cloud_);
}

void Camera::processCamImageData(const boost::shared_ptr<const CamImageDataMessage>& msg)
{
  images_caller_.addFunction(boost::bind(&Camera::processImages, this, msg));
}

void Camera::processImages(const boost::shared_ptr<const CamImageDataMessage>& msg)
{

  // LCM: (assumes mono)
  int n_colors=1;
  int isize = n_colors*msg->width*msg->height;

  ros::Time cam_time = convertTime(msg->timeStamp.getCurrentTime());
  msg_out_.utime = (int64_t) floor(cam_time.toNSec()/1000);
  msg_out_.width = msg->width;
  msg_out_.height = 2*msg->height;
  msg_out_.pixelformat =bot_core::image_t::PIXEL_FORMAT_GRAY;
  msg_out_.nmetadata =0;
  msg_out_.row_stride=n_colors*msg->width;
  msg_out_.size =2*n_colors*isize;

  msg_out_.data.resize( 2*isize);
  memcpy(&msg_out_.data[0], msg->leftImage, isize);
  memcpy(&msg_out_.data[isize], msg->rightImage, isize);

  //copy(msg->data.begin(), msg->data.end(), singleimage_data);
  //lcm_img.data.assign(singleimage_data, singleimage_data + ( n_colors*isize));
  lcm_publish_.publish("CAMERA", &msg_out_);
  /////////////////////////////////////////////////////////////


  if (left_cam_pub_.getNumSubscribers() > 0)
  {
    left_image_.header.frame_id = frame_id_;
    left_image_.header.stamp = convertTime(msg->timeStamp.getCurrentTime());

    left_image_.height = msg->height;
    left_image_.width  = msg->width;

    left_image_.encoding = "mono8";
    left_image_.is_bigendian = (htonl(1) == 1);
    left_image_.step = msg->width;
    left_image_.data.resize(msg->width * msg->height);
    memcpy(&left_image_.data[0], msg->leftImage, msg->width*msg->height);

    left_cam_info_.header = left_image_.header;

    left_cam_pub_.publish(left_image_, left_cam_info_);
  }

  if (right_cam_pub_.getNumSubscribers() > 0)
  {
    right_image_.header.frame_id = frame_id_;
    right_image_.header.stamp = convertTime(msg->timeStamp.getCurrentTime());

    right_image_.height = msg->height;
    right_image_.width  = msg->width;

    right_image_.encoding = "mono8";
    right_image_.is_bigendian = (htonl(1) == 1);
    right_image_.step = msg->width;
    right_image_.data.resize(msg->width * msg->height);
    memcpy(&right_image_.data[0], msg->rightImage, msg->width*msg->height);

    right_cam_info_.header = right_image_.header;

    right_cam_pub_.publish(right_image_, right_cam_info_);
  }

  image_diagnostics_.countStream();
}




void Camera::processCamStartStreamAck(const boost::shared_ptr<const CamStartStreamAckMessage>& msg)
{
  if (msg->status == 0)
    depth_diagnostics_.publish(SensorStatus::RUNNING);
  else
    ROS_ERROR("Failed to start depth stream.  Error code %d", msg->status);
}

void Camera::processCamStopStreamAck(const boost::shared_ptr<const CamStopStreamAckMessage>& msg)
{
  if (msg->status == 0)
    depth_diagnostics_.publish(SensorStatus::STOPPED);
  else
    ROS_ERROR("Failed to stop depth stream.  Error code %d", msg->status);
}

void Camera::processCamStartImageStreamAck(const boost::shared_ptr<const CamStartImageStreamAckMessage>& msg)
{
  if (msg->status == 0)
    image_diagnostics_.publish(SensorStatus::RUNNING);
  else
    ROS_ERROR("Failed to start images stream.  Error code %d", msg->status);
}

void Camera::processCamStopImageStreamAck(const boost::shared_ptr<const CamStopImageStreamAckMessage>& msg)
{
  if (msg->status == 0)
    image_diagnostics_.publish(SensorStatus::STOPPED);
  else
    ROS_ERROR("Failed to stop images stream.  Error code %d", msg->status);
}

void Camera::configureCB(multisense_ros::CameraConfig & config, uint32_t level)
{
  depth_diagnostics_.setExpectedRate(config.fps);
  image_diagnostics_.setExpectedRate(config.fps);

  CamControlMessage msg;
  msg.framesPerSecond = config.fps;
  if (config.auto_gain)
  {
    // Auto-Gain seems to be unsupported right now
    config.auto_gain = false;

    // msg.gain = -1.0;
    // msg.exposureTime = (uint32_t)(-1.0);  // ugly, but according to spec
  }

  msg.gain = config.gain;
  // Assuming that driver expects exposure time to be in us
  msg.exposureTime = config.exposure_time * 1000 * 1000;

  command_cam_control_.reset( new Command<CamControlMessage, CamControlAckMessage>(msg, driver_));
  command_cam_control_->run();
}

}

