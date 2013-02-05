#ifndef MULTISENSE_ROS_POINT_CLOUD2_CONVERTER_H
#define MULTISENSE_ROS_POINT_CLOUD2_CONVERTER_H

#include <sensor_msgs/CameraInfo.h>
#include <LibSensorPodCommunications/CamDataMessage.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/stereo_camera_model.h>

namespace multisense_ros
{

class PointCloud2Converter
{
public:
  void toPointCloud2(const sensor_msgs::CameraInfo& left_info,
                     const sensor_msgs::CameraInfo& right_info,
                     const CamDataMessage& cam_data,
                     sensor_msgs::PointCloud2& cloud_out) const;

private:
  mutable std::vector<float> disparity_buff_;
  mutable std::vector<cv::Vec3f> points_buff_;
  mutable image_geometry::StereoCameraModel model_;

};

}

#endif
