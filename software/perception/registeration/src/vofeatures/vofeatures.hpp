#ifndef VOFEATURES_HPP_
#define VOFEATURES_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <lcm/lcm-cpp.hpp>
#include <fovis/fovis.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "imagefeatures.hpp"
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include <lcmtypes/registeration.hpp>


class VoFeatures
{
public:
  VoFeatures(boost::shared_ptr<lcm::LCM> &lcm_, int image_width_, int image_height_);
  ~VoFeatures();

  void setFeatures(const fovis::FeatureMatch* matches, int num_matches, int64_t utime);
  void setImages(uint8_t *left_buf,uint8_t *right_buf){    
    left_buf_ = left_buf; 
    right_buf_ = right_buf;   
  }
  void setCameraPose(Eigen::Isometry3d ref_camera_pose){    
    ref_camera_pose_ = ref_camera_pose;
  }

  void sendFeatures();
private:
  boost::shared_ptr<lcm::LCM> lcm_;
  pointcloud_vis* pc_vis_;
  int image_width_;
  int image_height_;
  int output_counter_;
  
  void write_images();
  void write_ref_images();
  void writeFeatures(std::vector<ImageFeature> features);
  void sendFeatures(std::vector<ImageFeature> features);
  void sendFeaturesAsCollection(std::vector<ImageFeature> features, int vs_id);

  // All the incoming data and state:
  Eigen::Isometry3d ref_camera_pose_;
  int64_t utime_;
  // pointers to reference images: (only used of visual output):
  uint8_t *left_buf_, *right_buf_;
  std::vector<ImageFeature> featuresA;
  std::vector<int> featuresA_indices;


};

#endif
