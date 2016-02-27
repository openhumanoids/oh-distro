#ifndef ICP_3DREG_AND_PLOT_HPP_
#define ICP_3DREG_AND_PLOT_HPP_

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include "boost/filesystem.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pointcloud2_t.hpp>
#include <pronto_utils/conversions_lcm.hpp>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>

#include <icp-registration/icp_utils.h>
#include <icp-registration/cloud_accumulate.hpp>

class RegistrationConfig
{
  public:
    RegistrationConfig();

    const char *homedir;

    string cloud_name_A;
    string cloud_name_B;

    string configFile3D_;
    string initTrans_; //initial transformation for the reading cloud in the form [x,y,theta]

  private:
};

class Registration{
  public:
    Registration(boost::shared_ptr<lcm::LCM> &lcm_, const RegistrationConfig& reg_cfg_);
    
    ~Registration(){
    }    
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(int idx){ 
      if (idx == 0) 
        return reference_cloud_; 
      else
        return transformed_input_cloud_; }
    
    PM::TransformationParameters getTransform(){ return out_transform_; }
    
    void publishCloud(int cloud_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
    void getICPTransform(DP &cloud_in, DP &cloud_ref);

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    RegistrationConfig reg_cfg_;
    
    pronto_vis* pc_vis_ ;
  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_input_cloud_;
    PM::TransformationParameters out_transform_;     
};

#endif
