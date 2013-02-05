

#include <multisense_ros/point_cloud2_converter.h>

namespace multisense_ros
{

static bool isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}


void PointCloud2Converter::toPointCloud2(const sensor_msgs::CameraInfo& left_info,
                   const sensor_msgs::CameraInfo& right_info,
                   const CamDataMessage& cam_data,
                   sensor_msgs::PointCloud2& cloud_out) const
{
  static const int h = cam_data.height;
  static const int w = cam_data.width;

  model_.fromCameraInfo(left_info, right_info);

  // Convert Carnegie disparity format into floating point disparity. Store in local buffer
  cv::Mat_<uint16_t> disparity_orig(h, w, cam_data.disparityImage);
  disparity_buff_.resize(h * w);
  cv::Mat_<float> disparity(cam_data.height, cam_data.width, &(disparity_buff_[0]));
  disparity = disparity_orig / 16.0;

  // Allocate buffer for reprojection output
  points_buff_.resize(h * w);
  cv::Mat_<cv::Vec3f> points(h, w, &(points_buff_[0]));

  // Do the reprojection in open space
  static const bool handle_missing_values = true;
  cv::reprojectImageTo3D(disparity, points, model_.reprojectionMatrix(), handle_missing_values);

  // Copy data into PointCloud2 msg
  cloud_out.height = h;
  cloud_out.width  = w;

  cloud_out.height = h;
  cloud_out.width  = w;
  cloud_out.fields.resize (4);
  cloud_out.fields[0].name = "x";
  cloud_out.fields[0].offset = 0;
  cloud_out.fields[0].count = 1;
  cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[1].name = "y";
  cloud_out.fields[1].offset = 4;
  cloud_out.fields[1].count = 1;
  cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[2].name = "z";
  cloud_out.fields[2].offset = 8;
  cloud_out.fields[2].count = 1;
  cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_out.fields[3].name = "rgb";
  cloud_out.fields[3].offset = 12;
  cloud_out.fields[3].count = 1;
  cloud_out.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

  static const int STEP = 16;
  cloud_out.point_step = STEP;

  cloud_out.row_step = STEP * w;
  cloud_out.data.resize( cloud_out.row_step * h);
  cloud_out.is_dense = false; // there may be invalid points

  const float bad_point = std::numeric_limits<float>::quiet_NaN();

  for (int v = 0; v < h; ++v)
  {
    for (int u = 0; u < w; ++u)
    {
      int offset = (v*cloud_out.row_step) + (u*STEP);
      if (isValidPoint(points(v,u)))
      {
        // x,y,z,rgba
        memcpy (&cloud_out.data[offset + 0], &points(v,u)[0], sizeof (float));
        memcpy (&cloud_out.data[offset + 4], &points(v,u)[1], sizeof (float));
        memcpy (&cloud_out.data[offset + 8], &points(v,u)[2], sizeof (float));
        uint8_t g = cam_data.grayScaleImage[v*w + u];
        uint32_t rgb = (g << 16) | (g << 8) | g;
        memcpy (&cloud_out.data[offset +12], &rgb, sizeof (uint32_t));
      }
      else
      {
        memcpy (&cloud_out.data[offset + 0], &bad_point, sizeof (float));
        memcpy (&cloud_out.data[offset + 4], &bad_point, sizeof (float));
        memcpy (&cloud_out.data[offset + 8], &bad_point, sizeof (float));
        memcpy (&cloud_out.data[offset +12], &bad_point, sizeof (float));
      }
    }
  }
}

}
