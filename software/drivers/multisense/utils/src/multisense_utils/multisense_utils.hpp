#ifndef MULTISENSE_UTILS_HPP_
#define MULTISENSE_UTILS_HPP_

#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <image_utils/jpeg.h>

// Multisense Requires:
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/bot_core.hpp>

#include <multisense_image_utils/multisense_image_utils.hpp>


using namespace pcl;
using namespace pcl::io;

class multisense_utils {
  public:
    multisense_utils ();

    ////////////////////////////////////////////////////////////////////////
    void unpack_multisense(const uint8_t* depth_data, const uint8_t* color_data, int height, int width, cv::Mat_<double> repro_matrix, 
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, bool is_rgb = true, bool is_disparity = true);
    
    void unpack_multisense(const bot_core_images_t *msg, cv::Mat_<double> repro_matrix,
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    
    // CPP:
    void unpack_multisense(const bot_core::images_t *msg, cv::Mat_<double> repro_matrix, 
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    
    // an integer decimation factor:
    void set_decimate(int decimate_in){ decimate_ = decimate_in; };
    // Set to
    void set_remove_size(int size_threshold_in){ size_threshold_ = size_threshold_in; };

  private:

    int decimate_;

    // Disconnected/Small components filtering:
    int size_threshold_; // in pixels
    float depth_threshold_; // in m, not exposed yet
    
    // Multisense Compress:
    mutable std::vector<float> disparity_buf_;
    mutable std::vector<cv::Vec3f> points_buf_;  

    multisense_image_utils miu_;

    uint8_t* rgb_buf_ ;
    uint8_t* depth_buf_;
};

#endif
