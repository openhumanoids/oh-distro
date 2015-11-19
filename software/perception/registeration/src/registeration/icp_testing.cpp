// cddrc
// drc-icp-testing externals/libpointmatcher/examples/data/car_cloud400.csv externals/libpointmatcher/examples/data/car_cloud401.csv

/*

Copyright (c) 2010--2012,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/pronto/pointcloud2_t.hpp>
//#include <pronto_utils/conversions_lcm.hpp>

#include "cloud_accumulate.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>

using namespace std;

const char *homedir;
boost::shared_ptr<lcm::LCM> lcm_;
pronto_vis* pc_vis_;

void convertTo2DDataPoints(PointMatcher<float>::DataPoints &loaded_cloud);
void fromDataPointsToPCL(PointMatcher<float>::DataPoints &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);
void publishCloud(int cloud_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_topub);

/**
  * Code example for ICP taking 2 points clouds (2D or 3D) relatively close 
  * and computing the transformation between them.
  */
int main(int argc, char *argv[])
{
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  //Set up LCM channel and config for visualization
  boost::shared_ptr<lcm::LCM> lcm_(new lcm::LCM);
  if(!lcm_->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  bool reset = 0;  
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000, "Pose - Null", 5, reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60001, "Cloud_Ref - Null", 1, reset, 60000, 1, {0.0, 0.1, 0.0}) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60002, "Cloud_In - Null", 1, reset, 60000, 1, {0.0, 0.0, 1.0}) );  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60003, "Cloud_Result - Null", 1, reset, 60000, 1, {1.0, 0.0, 0.0}) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60004, "Scan_Ref - Null", 1, reset, 60000, 1, {0.0, 0.1, 0.0}) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60005, "Scan_In - Null", 1, reset, 60000, 1, {0.0, 0.0, 1.0}) );  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60006, "Scan_Result - Null", 1, reset, 60000, 1, {1.0, 0.0, 0.0}) );
  

  typedef PointMatcher<float> PM;
  typedef PM::DataPoints DP;
  
  string vtk_fname_A, vtk_fname_B;

  //=================================
  // TRANSFORM 3D CLOUD
  //=================================
  // Load point clouds
  vtk_fname_A.append(homedir);
  vtk_fname_A.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_00.vtk");
  //vtk_fname_A.append("/main-distro/software/externals/libpointmatcher/examples/data/car_cloud400.csv");
  vtk_fname_B.append(homedir);
  vtk_fname_B.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_08.vtk");
  //vtk_fname_B.append("/main-distro/software/externals/libpointmatcher/examples/data/car_cloud401.csv");
  DP ref(DP::load(vtk_fname_A));
  DP data(DP::load(vtk_fname_B));

  // Transform input clouds to pcl::PointCloud<pcl::PointXYZRGB> and publish for visualization
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB> ());
  fromDataPointsToPCL(ref, *cloudA);
  publishCloud(60001, cloudA);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB> ());
  fromDataPointsToPCL(data, *cloudB);
  publishCloud(60002, cloudB);

  // Create the default ICP algorithm
  PM::ICP icp;
  
  // See the implementation of setDefault() to create a custom ICP algorithm
  icp.setDefault();

  // Compute the transformation to express data in ref
  PM::TransformationParameters T = icp(data, ref);

  // Transform data to express it in ref
  DP data_out(data);
  icp.transformations.apply(data_out, T);
  
  // Transform output cloud to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_res (new pcl::PointCloud<pcl::PointXYZRGB> ());
  fromDataPointsToPCL(data_out, *cloud_res);
  // Publish cloud to see the results
  publishCloud(60003, cloud_res);

  cout << "Final 3D transformation:" << endl << T << endl;

  //=================================
  // TRANSFORM CORRESPONDING 2D CLOUD
  //=================================

  vtk_fname_A.clear();
  vtk_fname_A.append(homedir);
  vtk_fname_A.append("/logs/multisenselog__2015-11-16/planar_scans/scan_00.vtk");
  vtk_fname_B.clear();
  vtk_fname_B.append(homedir);
  vtk_fname_B.append("/logs/multisenselog__2015-11-16/planar_scans/scan_08.vtk");
  DP ref2(DP::load(vtk_fname_A));
  DP data2(DP::load(vtk_fname_B));

  /*
  Features is an Eigen matrix typically containing the coordinates of the points which form the cloud. 
  Each column corresponds to a point in the cloud. 
  The rows correspond to the dimensions of the points in homogeneous coordinates. 
  For 2D point clouds, there will thus be 3 rows and for 4 rows for 3D point clouds.
  */
  convertTo2DDataPoints(ref2);
  convertTo2DDataPoints(data2);

  // Transform input clouds to pcl::PointCloud<pcl::PointXYZRGB> and publish for visualization
  fromDataPointsToPCL(ref2, *cloudA);
  publishCloud(60004, cloudA);
  fromDataPointsToPCL(data2, *cloudB);
  publishCloud(60005, cloudB);
  
  // See the implementation of setDefault() to create a custom ICP algorithm
  icp.setDefault();

  // Compute the transformation to express data in ref
  T = icp(data2, ref2);

  // Transform data to express it in ref
  DP data_out2(data2);
  icp.transformations.apply(data_out2, T);
  
  // Transform output cloud to pcl::PointCloud<pcl::PointXYZRGB>
  fromDataPointsToPCL(data_out2, *cloud_res);
  // Publish cloud to see the results
  publishCloud(60006, cloud_res);

  cout << "Final 2D transformation:" << endl << T << endl;
  
  return 0;
}

void convertTo2DDataPoints(PointMatcher<float>::DataPoints &loaded_cloud)
{
  Eigen::MatrixXf ref2D(3,loaded_cloud.getNbPoints());
  for (int i = 0; i < loaded_cloud.getNbPoints(); i++) {
    ref2D(0, i) = (loaded_cloud.features.col(i))[0];
    ref2D(1, i) = (loaded_cloud.features.col(i))[1];
    ref2D(2, i) = 1;
  }  
  loaded_cloud.addFeature("x", ref2D.row(0));
  loaded_cloud.addFeature("y", ref2D.row(1));
  loaded_cloud.addFeature("z", ref2D.row(2));
  loaded_cloud.removeFeature("pad");
}

void fromDataPointsToPCL(PointMatcher<float>::DataPoints &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{
  cloud_out.points.resize(cloud_in.getNbPoints());
  for (int i = 0; i < cloud_in.getNbPoints(); i++) {
    cloud_out.points[i].x = (cloud_in.features.col(i))[0];
    cloud_out.points[i].y = (cloud_in.features.col(i))[1];
    cloud_out.points[i].z = (cloud_in.features.col(i))[2];
    //cout << "i=" << i << " " << cloud_out.points[i].x << " " << cloud_out.points[i].y << " " << cloud_out.points[i].z << endl;
  }  
  cloud_out.width = cloud_out.points.size();
  cloud_out.height = 1;
}

void publishCloud(int cloud_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_toPub)
{
  Isometry3dTime null_T = Isometry3dTime(1, Eigen::Isometry3d::Identity());
  pc_vis_->pose_to_lcm_from_list(60000, null_T);
  pc_vis_->ptcld_to_lcm_from_list(cloud_id, *cloud_toPub, 1, 1);
}



