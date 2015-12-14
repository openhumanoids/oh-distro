// Start. Get 2 point clouds (either from user or default).
// Perform registration (input cloud aligned to reference).
// Convert to octomap, blur and save to file.
// Republish the octomap.
// icp-octrees-diff -h

#include <sstream>      // std::stringstream

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <icp-registration/icp_3Dreg_and_plot.hpp>
#include <icp-registration/clouds_io_utils.h>
#include "pointmatcher/PointMatcher.h"

#include "convert_octomap.hpp"

//Compare
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

struct AppConfig
{
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, RegistrationConfig reg_cfg_, 
        ConvertOctomapConfig co_cfg_, AppConfig app_cfg);
    
    ~App(){
    }  

    RegistrationConfig reg_cfg_;
    ConvertOctomapConfig co_cfg_;
    AppConfig app_cfg_;
    
    boost::shared_ptr<lcm::LCM> lcm_;
    Registration* registr_; 
    ConvertOctomap* convert_; 

    bool do_convert_cloud_; // true if the pt cloud is ready to be converted to octomap
    bool do_republish_; // true if an octree is ready  

  private:
    
};

App::App(boost::shared_ptr< lcm::LCM >& lcm_, RegistrationConfig reg_cfg_, 
         ConvertOctomapConfig co_cfg_, AppConfig app_cfg_) : lcm_(lcm_), 
         reg_cfg_(reg_cfg_), co_cfg_(co_cfg_),
         app_cfg_(app_cfg_){
  registr_ = new Registration(lcm_, reg_cfg_);
  std::cout << "Clouds matching at launch.\n"; 
  convert_ = new ConvertOctomap(lcm_, co_cfg_); 
  std::cout << "Clouds to octrees conversion at launch.\n";

  do_convert_cloud_ = false;
  do_republish_ = false;
}

OcTree* doConversion(App* app, int cloud_idx); // NOTE: accepted cloud indexes are 0 (reference) and 1 (input).
void convertCloudPclToPronto(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pronto::PointCloud &cloud_out);
int validateArgs(const int argc, const char *argv[], RegistrationConfig& reg_cfg, ConvertOctomapConfig& co_cfg);
void usage(const char *argv[]);

void printChanges(OcTree& tree);

int main(int argc, const char *argv[])
{
  // Init Default
  RegistrationConfig reg_cfg;
  reg_cfg.configFile3D_.clear();
  reg_cfg.initTrans_.clear();
  reg_cfg.initTrans_.append("0,0,0");
  ConvertOctomapConfig co_cfg;
  co_cfg.octomap_resolution = 0.1; // was always 0.1 for mav and atlas
  co_cfg.blur_sigma = 0.1; // default was .5
  co_cfg.blur_map = false;
  AppConfig app_cfg;

  const int ret = validateArgs(argc, argv, reg_cfg, co_cfg);

  if (ret == -1)
    return ret;

  if (reg_cfg.cloud_name_A.empty())
  {
    reg_cfg.cloud_name_A.append(reg_cfg.homedir);
    reg_cfg.cloud_name_A.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_00.vtk");	
  }
  if (reg_cfg.cloud_name_B.empty())
  {    
  	reg_cfg.cloud_name_B.append(reg_cfg.homedir);
    reg_cfg.cloud_name_B.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_01.vtk");	
  }

  //std::cout << "Blur sigma: " << co_cfg.blur_sigma << "\n";

  //Set up LCM channel for visualization
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  App* app = new App(lcm, reg_cfg, co_cfg, app_cfg);     
  
  // Load point clouds from file
  DP ref = DP::load(reg_cfg.cloud_name_A);
  DP data = DP::load(reg_cfg.cloud_name_B);
  
  //=================================
  // TRANSFORM 3D CLOUD
  //=================================

  app->registr_->getICPTransform(data, ref);
  
  PM::TransformationParameters T = app->registr_->getTransform();
  cout << "3D Transformation:" << endl << T << endl;

  //=================================
  // CONVERT CLOUD TO OCTREE
  //================================= 

  app->do_convert_cloud_ = true; 

  if ( app->do_convert_cloud_ ) {

    OcTree* treeA = doConversion(app, 0);
    cout << "========================================" << endl;
    cout << "Changes A:" << endl;
    printChanges(*treeA);
    
    //app->convert_->clearTree();
    OcTree* treeB = doConversion(app, 1);
    cout << "========================================" << endl;
    cout << "Changes B:" << endl;
    printChanges(*treeB);
  }
  
  return 0;
}

void printChanges(OcTree& tree){
  unsigned int changedOccupied = 0;
  unsigned int changedFree = 0;
  unsigned int actualOccupied = 0;
  unsigned int actualFree = 0;
  unsigned int missingChanged = 0;

  tree.expand();

  // iterate through the changed nodes
  KeyBoolMap::const_iterator it;
  for (it=tree.changedKeysBegin(); it!=tree.changedKeysEnd(); it++) {
    OcTreeNode* node = tree.search(it->first);
    if (node != NULL) {
      if (tree.isNodeOccupied(node)) {
        changedOccupied += 1;
      }
      else {
        changedFree += 1;
      }
    } else {
      missingChanged +=1;
    }
  }


  // iterate through the entire tree
  for(OcTree::tree_iterator it=tree.begin_tree(),
      end=tree.end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      if (tree.isNodeOccupied(*it)) {
        actualOccupied += 1;
      }
      else {
        actualFree += 1;
      }
    }
  }
  
  cout << "Change detection: " << changedOccupied << " occ; " << changedFree << " free; "<< missingChanged << " missing" << endl;
  cout << "Number of changed nodes: " << changedOccupied+changedFree << endl;
  cout << "Actual: " << actualOccupied << " occ; " << actualFree << " free; " << endl;
  cout << "Number of nodes: " << actualOccupied+actualFree << endl;

  tree.prune();
}

OcTree* doConversion(App* app, int cloud_idx)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud = app->registr_->getCloud(cloud_idx);

  pronto::PointCloud* pronto_cloud (new pronto::PointCloud ());
  convertCloudPclToPronto(*pcl_cloud, *pronto_cloud);

  cout << "====================================================" << endl;
  cout << "Processing cloud with " << pronto_cloud->points.size()
       << " points" << endl;
  string current_cloud;  
  if (cloud_idx == 0)    
    current_cloud = "OCTOMAP_REF";
  else
    current_cloud = "OCTOMAP_IN";

  app->convert_->doWork(pronto_cloud, current_cloud);
      
  cout << "Finished processing." << endl;      
           
  app->do_republish_ = true;
  app->do_convert_cloud_ = false;

  if (app->do_republish_){
    //std::cout << "Republishing unblurred octomap.\n";
    //fprintf(stderr, "r");
    app->convert_->publishOctree(app->convert_->getTree(),current_cloud);
  }

  return app->convert_->getTree();
}

void convertCloudPclToPronto(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pronto::PointCloud &cloud_out){
  int npts = cloud.points.size();
  cloud_out.points.resize(npts);
  for(int j=0; j<npts; j++) {
    cloud_out.points[j].x = cloud.points[j].x;
    cloud_out.points[j].y = cloud.points[j].y;
    cloud_out.points[j].z = cloud.points[j].z;
    cloud_out.points[j].r = cloud.points[j].r;
    cloud_out.points[j].g = cloud.points[j].g;
    cloud_out.points[j].b = cloud.points[j].b;
  }
}

// Make sure that the command arguments make sense
int validateArgs(const int argc, const char *argv[], RegistrationConfig& reg_cfg, ConvertOctomapConfig& co_cfg)
{
  const int endOpt(argc);

  for (int i = 1; i < endOpt; i += 2)
  {
    const string opt(argv[i]);
    if (i + 1 > endOpt)
    {
      cerr << "Incorrect use of option " << opt << ", usage:"; usage(argv); exit(1);
    }
    if (opt == "-c" || opt == "-config") {
      reg_cfg.configFile3D_.append(getenv("DRC_BASE"));
      reg_cfg.configFile3D_.append("/software/perception/registeration/filters_config/");
      reg_cfg.configFile3D_.append(argv[i+1]);
    }
    else if (opt == "-i" || opt == "--initT") {
      reg_cfg.initTrans_.clear();
      reg_cfg.initTrans_ = argv[i+1];
    }
    else if (opt == "-a" || opt == "--reference") {
      reg_cfg.cloud_name_A.append(argv[i+1]);
    }
    else if (opt == "-b" || opt == "--input") {
      reg_cfg.cloud_name_B.append(argv[i+1]);
    }
    else if (opt == "-R" || opt == "--octomapRes") {
      co_cfg.octomap_resolution = atof(argv[i+1]);
    }
    else if (opt == "-B" || opt == "--blurSigma") {
      co_cfg.blur_sigma = atof(argv[i+1]);
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
  cerr << "* To run ICP registration:" << endl;
  cerr << "  " << argv[0] << " [OPTIONS]" << endl;
  cerr << endl;
  cerr << "OPTIONS can be a combination of:" << endl;
  cerr << "-c or --config --> YAML_3DCLOUD_CONFIG_FILE  Load the config from a YAML file located in filters_config (default: default parameters)" << endl;
  cerr << "-i or --initT --> [x,y,theta]  Initial transformation applyed to input cloud (default: 0,0,0)" << endl;
  cerr << "-a or --reference --> Reference cloud name  Load the .vtk file from current folder (default: multisense_00.vtk)" << endl;
  cerr << "-b or --input --> Reference cloud name  Load the .vtk file from current folder (default: multisense_01.vtk)" << endl;
  cerr << "-R or --octomapRes --> Resolution of underlying octomap (default: 0.1)" << endl;
  cerr << "-B or --blurSigma --> Radius of the blur kernel (default: 0.1)" << endl;
  cerr << endl;
}