// cddrc
// drc-icp-testing

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include "boost/filesystem.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/pronto/pointcloud2_t.hpp>
#include <pronto_utils/conversions_lcm.hpp>

#include "cloud_accumulate.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>

#include "icp_utils.h"

using namespace std;

const char *homedir;

boost::shared_ptr<lcm::LCM> lcm_;
pronto_vis* pc_vis_;

string configFile2D_, configFile3D_;
string initTrans_;

void getICPTransform(DP &cloud_in, DP &cloud_ref);
void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);
void publishCloud(int cloud_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_topub);
void applyRigidTransform(DP &pointCloud);

int validateArgs(const int argc, const char *argv[]);
void usage(const char *argv[]);

/**
  * Code for ICP reading : 1) two 3D point clouds relatively close 
  * and computing the transformation between them. 2) two 2D point clouds 
  * relatively close and computing the transformation between them.
  */

int main(int argc, const char *argv[])
{
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  //Set up LCM channel and config for visualization
  boost::shared_ptr<lcm::LCM> lcm_(new lcm::LCM);
  if(!lcm_->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // Init Default
  initTrans_.append("0,0,0");

  const int ret = validateArgs(argc, argv);

  if (ret == -1)
    return ret;

  /////////////////////////////////////////////////////// Set up pronto visualizer
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

  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60007, "Cloud_Trans - Null", 1, reset, 60000, 1, {0.0, 1.0, 0.0}) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60008, "Scan_Trans - Null", 1, reset, 60000, 1, {0.0, 1.0, 0.0}) );
  ////////////////////////////////////////////////////////////////////////////////
  
  string cloud_name_A, cloud_name_B, scan_name_A, scan_name_B;
  DP ref, data, ref2, data2;

  cloud_name_A.append(homedir);
  cloud_name_A.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_05.vtk");
  //cloud_name_A.append("/logs/multisenselog__2015-11-16/tmp/multisense_00.vtk");
  //cloud_name_A.append("/main-distro/software/externals/libpointmatcher/examples/data/car_cloud400.csv");
  cloud_name_B.append(homedir);
  cloud_name_B.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_08.vtk");
  //cloud_name_B.append("/logs/multisenselog__2015-11-16/tmp/multisense_08.vtk");
  //cloud_name_B.append("/main-distro/software/externals/libpointmatcher/examples/data/car_cloud401.csv");

  scan_name_A.append(homedir);
  scan_name_A.append("/logs/multisenselog__2015-11-16/planar_scans/scan_03.csv");
  //scan_name_A.append("/logs/multisenselog__2015-11-16/tmp/scan_00.csv");
  //scan_name_A.append("/main-distro/software/externals/libpointmatcher/examples/data/2D_twoBoxes.csv");
  scan_name_B.append(homedir);
  scan_name_B.append("/logs/multisenselog__2015-11-16/planar_scans/scan_05.csv");
  //scan_name_B.append("/logs/multisenselog__2015-11-16/tmp/scan_08.csv");
  //scan_name_B.append("/main-distro/software/externals/libpointmatcher/examples/data/2D_oneBox.csv");

  // Load point clouds from file
  ref = DP::load(cloud_name_A);
  data = DP::load(cloud_name_B);

  ref2 = DP::load(scan_name_A);
  data2 = DP::load(scan_name_B);
  
  //=================================
  // TRANSFORM 3D CLOUD
  //=================================

  getICPTransform(data, ref);

  //=================================
  // TRANSFORM 2D CLOUD
  //=================================

  //getICPTransform(data2, ref2);
  
  return 0;
}

//========================================================
// FUNCTIONS
//========================================================

// Make sure that the command arguments make sense
int validateArgs(const int argc, const char *argv[])
{
  if (argc == 1)
  {
    cerr << "Loading clouds... default options applied.\n";
    return 1;
  }
 
  const int endOpt(argc);

  for (int i = 1; i < endOpt; i += 2)
  {
    const string opt(argv[i]);
    if (i + 1 > endOpt)
    {
      cerr << "Missing value for option " << opt << ", usage:"; usage(argv); exit(1);
    }
    if (opt == "--config2D") {
      configFile2D_.append(homedir);
      configFile2D_.append("/main-distro/software/perception/registeration/filters_config/");
      configFile2D_.append(argv[i+1]);
    }
    else if (opt == "--config3D") {
      configFile3D_.append(homedir);
      configFile3D_.append("/main-distro/software/perception/registeration/filters_config/");
      configFile3D_.append(argv[i+1]);
    }
    else if (opt == "--initTrans") {
      initTrans_.clear();
      initTrans_ = argv[i+1];
    }
    else if (opt == "-h")
    {
      cerr << "Usage:";
      usage(argv);
      return -1;
    }
    else
    {
      cerr << "Unknown option " << opt << ", usage:"; usage(argv); exit(1);
    }
  }
  return 0;
}


// Dump command-line help
void usage(const char *argv[])
{
  //TODO: add new options --isTransfoSaved, --initTranslation, --initRotation
  cerr << endl << endl;
  cerr << "* To run ICP:" << endl;
  cerr << "  " << argv[0] << " [OPTIONS]" << endl;
  cerr << endl;
  cerr << "OPTIONS can be a combination of:" << endl;
  cerr << "--config2D YAML_2DCLOUD_CONFIG_FILE  Load the config from a YAML file located in filters_config (default: default parameters)" << endl;
  cerr << "--config3D YAML_3DCLOUD_CONFIG_FILE  Load the config from a YAML file located in filters_config (default: default parameters)" << endl;
  cerr << "--initTrans [x,y,theta]  Add an initial transformation before applying ICP (default: 0,0,0)" << endl;
  cerr << endl;
}


void getICPTransform(DP &cloud_in, DP &cloud_ref)
{
  // Transform input clouds to pcl::PointCloud<pcl::PointXYZRGB> and publish for visualization
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB> ());
  fromDataPointsToPCL(cloud_ref, *cloudA);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB> ());
  fromDataPointsToPCL(cloud_in, *cloudB);

  // Create the default ICP algorithm
  PM::ICP icp;

  int cloudDimension = cloud_ref.getEuclideanDim();
  
  if (!(cloudDimension == 2 || cloudDimension == 3)) 
  {
    cerr << "Invalid input point clouds dimension." << endl;
    exit(1);
  }

  if (cloudDimension == 2)
  {
    // ICP chain configuration: check if prefiltering required
    if (configFile2D_.empty())
    {
      // See the implementation of setDefault() to create a custom ICP algorithm
      icp.setDefault();
    }
    else
    {
      // load YAML config
      ifstream ifs(configFile2D_.c_str());
      if (!ifs.good())
      {
        cerr << "Cannot open config file " << configFile2D_ << endl; exit(1);
      }
      icp.loadFromYaml(ifs);
      cerr << "Loaded pre-filtering chain from yaml..." << endl;
    }
  }
  else
  {
    // ICP chain configuration: check if prefiltering required
    if (configFile3D_.empty())
    {
      // See the implementation of setDefault() to create a custom ICP algorithm
      icp.setDefault();
    }
    else
    {
      // load YAML config
      ifstream ifs(configFile3D_.c_str());
      if (!ifs.good())
      {
        cerr << "Cannot open config file " << configFile3D_ << endl; exit(1);
      }
      icp.loadFromYaml(ifs);
      cerr << "Loaded pre-filtering chain from yaml..." << endl;
    }
  }

  // Apply rigid transformation (just a "visually good" approximation of the transformation 
  // between ref and input clouds) to escape local minima
  PM::TransformationParameters initTransfo = parseTransformation(initTrans_, cloudDimension);

  PM::Transformation* rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

  if (!rigidTrans->checkParameters(initTransfo)) {
    cerr << endl
       << "Initial transformation is not rigid, identity will be used."
       << endl;
    initTransfo = PM::TransformationParameters::Identity(
          cloudDimension+1,cloudDimension+1);
  }

  DP initializedData = rigidTrans->compute(cloud_in, initTransfo);

  // Compute the transformation to express data in ref
  PM::TransformationParameters T = icp(initializedData, cloud_ref);
  cout << "Match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;

  // Transform data to express it in ref
  DP data_out(initializedData);
  icp.transformations.apply(data_out, T);

  // Transform output cloud to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_res (new pcl::PointCloud<pcl::PointXYZRGB> ());
  fromDataPointsToPCL(data_out, *cloud_res);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZRGB> ());
  fromDataPointsToPCL(initializedData, *cloud_trans);

  // Publish clouds to see the results
  if (cloudDimension == 3)
  {
    publishCloud(60001, cloudA);
    publishCloud(60002, cloudB);
    publishCloud(60003, cloud_res);
    publishCloud(60007, cloud_trans);
    cout << "Final 3D transformation:" << endl << T << endl;
  }
  else
  {
    publishCloud(60004, cloudA);
    publishCloud(60005, cloudB);
    publishCloud(60006, cloud_res); 
    publishCloud(60008, cloud_trans);
    cout << "Final 2D transformation:" << endl << T << endl;
  }

  //======================== Errors............... ==========================
  //computeCloudsDistance (icp, cloud_ref, data_out);
  //=========================================================================
}


void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
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

