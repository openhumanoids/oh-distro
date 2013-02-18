#ifndef MULTISENSE_ROS_CAMERA_H
#define MULTISENSE_ROS_CAMERA_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <multisense_driver/multisense_driver.h>
#include <multisense_ros/tools.h>
#include <multisense_ros/state_publisher.h>
#include <multisense_ros/point_cloud2_converter.h>
#include <multisense_ros/function_caller.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <multisense_ros/CameraConfig.h>

#include <LibSensorPodCommunications/CamDataMessage.h>
#include <LibSensorPodCommunications/CamImageDataMessage.h>
#include <LibSensorPodCommunications/CamConfigMessage.h>
#include <LibSensorPodCommunications/CamStartStreamAckMessage.h>
#include <LibSensorPodCommunications/CamStopStreamAckMessage.h>
#include <LibSensorPodCommunications/CamStartImageStreamAckMessage.h>
#include <LibSensorPodCommunications/CamStopImageStreamAckMessage.h>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>
#include <lcm/lcm-cpp.hpp>

namespace multisense_ros
{

class Camera
{
public:
  Camera(multisense_driver::MultisenseDriver* driver);

private:

  // ROS topic callbacks
  void connectDepthCB();
  void disconnectDepthCB();
  void connectImageCB();
  void disconnectImageCB();

  void stopDepthStream();
  void stopImageStream();

  // Multisense Driver callbacks
  void processCamConfig(const boost::shared_ptr<const CamConfigMessage>& msg);
  void processCamData(const boost::shared_ptr<const CamDataMessage>& msg);
  void processCamImageData(const boost::shared_ptr<const CamImageDataMessage>& msg);
  void processCamStartStreamAck(const boost::shared_ptr<const CamStartStreamAckMessage>& msg);
  void processCamStopStreamAck(const boost::shared_ptr<const CamStopStreamAckMessage>& msg);
  void processCamStartImageStreamAck(const boost::shared_ptr<const CamStartImageStreamAckMessage>& msg);
  void processCamStopImageStreamAck(const boost::shared_ptr<const CamStopImageStreamAckMessage>& msg);

  // Worker functions called by FunctionCallers in separate threads
  void processDepthImage(const boost::shared_ptr<const CamDataMessage>& msg);
  void processPointCloud(const boost::shared_ptr<const CamDataMessage>& msg);
  void processImages(const boost::shared_ptr<const CamImageDataMessage>& msg);

  // Dynamic Reconfigure
  void configureCB(multisense_ros::CameraConfig & config, uint32_t level);

  multisense_driver::MultisenseDriver* driver_;
  ros::NodeHandle stereo_nh_;
  ros::NodeHandle diagnostics_nh_;
  ros::NodeHandle left_nh_;
  ros::NodeHandle right_nh_;


  image_transport::ImageTransport  left_transport_;
  image_transport::ImageTransport  right_transport_;
  image_transport::ImageTransport  depth_transport_;
  PointCloud2Converter point_cloud_converter_;

  image_transport::CameraPublisher left_cam_pub_;
  image_transport::CameraPublisher right_cam_pub_;
  image_transport::CameraPublisher depth_cam_pub_;
  ros::Publisher point_cloud_pub_;

  boost::shared_ptr<multisense_ros::CommandBase> command_cam_data_;
  boost::shared_ptr<multisense_ros::CommandBase> command_cam_image_data_;

  multisense_driver::MultisenseSubscriber<CamDataMessage> cam_data_sub_;
  multisense_driver::MultisenseSubscriber<CamImageDataMessage> cam_image_data_sub_;

  boost::shared_ptr<multisense_ros::CommandBase> command_cam_get_config_;
  bool got_cam_info_;
  std::string frame_id_;
  sensor_msgs::CameraInfo left_cam_info_;
  sensor_msgs::CameraInfo right_cam_info_;

  multisense_ros::FunctionCaller depth_image_caller_;
  multisense_ros::FunctionCaller point_cloud_caller_;
  multisense_ros::FunctionCaller images_caller_;

  // Diagnostics
  multisense_ros::StatePublisher depth_diagnostics_, image_diagnostics_, config_diagnostics_;

  // Dynamic Reconfigure
  dynamic_reconfigure::Server<multisense_ros::CameraConfig> reconfigure_server_;
  boost::shared_ptr<multisense_ros::CommandBase> command_cam_control_;

  // Store outgoing messages for efficiency
  mutable sensor_msgs::Image depth_image_;
  mutable sensor_msgs::Image left_image_;
  mutable sensor_msgs::Image right_image_;
  mutable sensor_msgs::PointCloud2 point_cloud_;


  lcm::LCM lcm_publish_ ;
  //
  bot_core::image_t stereo_msg_out_;
  // this message bundles both together:
  multisense::images_t multisense_msg_out_;
  bot_core::image_t disparity_msg_out_;
  bot_core::image_t left_msg_out_;
  bot_core::image_t right_msg_out_;
};

}

#endif
