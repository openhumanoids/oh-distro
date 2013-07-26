#ifndef kintinuous_pub_HPP_
#define kintinuous_pub_HPP_

#include <iostream>
#include <vector>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core_image_t.h"
#include "lcmtypes/multisense_images_t.h"
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>



class kintinuous_pub {
  public:
    kintinuous_pub ();
    
    void setTimestamp(int64_t current_utime_in){ 
      current_utime = current_utime_in;
    }


    void sendImages(uint8_t * colorData, uint16_t * depthData);
    void writeToFile(uint16_t * depthData);

    void sendPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    boost::shared_ptr<lcm::LCM> lcm_;

    int width_, height_, npixels_;
    int ncolors_, ndepthbytes_;

    multisense::images_t lcm_fused_;
    bot_core::image_t lcm_left_;
    bot_core::image_t lcm_depth_;

    int64_t current_utime;


};






#endif
