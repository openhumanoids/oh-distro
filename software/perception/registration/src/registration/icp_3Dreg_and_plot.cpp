#include "icp_3Dreg_and_plot.hpp"

RegistrationConfig::RegistrationConfig(){
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  cloud_name_A.clear();
  cloud_name_A.clear();
  configFile3D_.clear();
  initTrans_.append("0,0,0");
}

Registration::Registration(boost::shared_ptr<lcm::LCM> &lcm_, const RegistrationConfig& reg_cfg_):
    lcm_(lcm_), reg_cfg_(reg_cfg_){
  
  //================ Set up pronto visualizer ===============
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
  //==========================================================

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ref_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  reference_cloud_ = cloud_ref_ptr;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  transformed_input_cloud_ = cloud_in_ptr;
}

void Registration::getICPTransform(DP &cloud_in, DP &cloud_ref)
{
  // Transform input clouds into a pcl PointXYZRGB and publish for visualization
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB> ());
  fromDataPointsToPCL(cloud_ref, *reference_cloud_);
  fromDataPointsToPCL(cloud_in, *cloudB);

  // Create the default ICP algorithm
  PM::ICP icp;

  int cloudDimension = cloud_ref.getEuclideanDim();
  
  if (!(cloudDimension == 3)) 
  {
    cerr << "Invalid input point clouds dimension." << endl;
    exit(1);
  }

  // ICP chain configuration: check if prefiltering required
  if (reg_cfg_.configFile3D_.empty())
  {
    // See the implementation of setDefault() to create a custom ICP algorithm
    icp.setDefault();
  }
  else
  {
    // load YAML config
    ifstream ifs(reg_cfg_.configFile3D_.c_str());
    if (!ifs.good())
    {
      cerr << "Cannot open config file " << reg_cfg_.configFile3D_ << endl; exit(1);
    }
    icp.loadFromYaml(ifs);
    cerr << "Loaded pre-filtering chain from yaml..." << endl;
  }

  // Apply rigid transformation (just a "visually good" approximation of the transformation 
  // between ref and input clouds) to escape local minima
  PM::TransformationParameters initT = parseTransformationDeg(reg_cfg_.initTrans_, cloudDimension);

  PM::Transformation* rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

  if (!rigidTrans->checkParameters(initT)) {
    cerr << endl
      << "Initial transformation is not rigid, identity will be used." << endl;
    initT = PM::TransformationParameters::Identity(cloudDimension+1,cloudDimension+1);
  }
  else
    cout << "Initialization: " << reg_cfg_.initTrans_ << endl;

  // Compute the transformation to express input in ref
  PM::TransformationParameters T = icp(cloud_in, cloud_ref, initT);
  cout << "Match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;

  // Transform input to express it in ref
  DP data_out(cloud_in);
  icp.transformations.apply(data_out, T);

  // Plot input after initialization and after final transformation
  fromDataPointsToPCL(data_out, *transformed_input_cloud_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZRGB> ());
  DP initializedInput = rigidTrans->compute(cloud_in, initT);
  fromDataPointsToPCL(initializedInput, *cloud_trans);

  // Publish clouds: plot in pronto visualizer
  publishCloud(60001, reference_cloud_);
  publishCloud(60002, cloudB);
  publishCloud(60003, transformed_input_cloud_);
  publishCloud(60007, cloud_trans);

  out_transform_ = T;
}

void Registration::publishCloud(int cloud_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  Isometry3dTime null_T = Isometry3dTime(1, Eigen::Isometry3d::Identity());
  pc_vis_->pose_to_lcm_from_list(60000, null_T);
  pc_vis_->ptcld_to_lcm_from_list(cloud_id, *cloud, 1, 1);
}



