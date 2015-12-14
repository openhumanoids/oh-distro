// drc-icp-plot

// Program for 3D clouds alignment
// using libpointmatcher (library for ICP).
//
// Options: - take 2 clouds from user (or default) and give 
//            transformation between them (-a reference, -b input)

#include <sstream>      // std::stringstream

#include "icp_3Dreg_and_plot.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "clouds_io_utils.h"

struct AppConfig
{
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, 
        RegistrationConfig reg_cfg_, AppConfig app_cfg);
    
    ~App(){
    }  

    RegistrationConfig reg_cfg_;
    AppConfig app_cfg_;
    
    boost::shared_ptr<lcm::LCM> lcm_;
    Registration* registr_;  

  private:
    
};

App::App(boost::shared_ptr< lcm::LCM >& lcm_, 
         RegistrationConfig reg_cfg_, AppConfig app_cfg_) : lcm_(lcm_), 
         reg_cfg_(reg_cfg_),
         app_cfg_(app_cfg_){

  registr_ = new Registration(lcm_, reg_cfg_);
  std::cout << "Clouds matching at launch.\n";  
}

int validateArgs(const int argc, const char *argv[], RegistrationConfig& reg_cfg);
void usage(const char *argv[]);

int main(int argc, const char *argv[])
{
  // Init Default
  RegistrationConfig reg_cfg;
  reg_cfg.configFile3D_.clear();
  reg_cfg.initTrans_.clear();
  reg_cfg.initTrans_.append("0,0,0");
  AppConfig app_cfg;

  const int ret = validateArgs(argc, argv, reg_cfg);

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

  //Set up LCM channel for visualization
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  App* app = new App(lcm, reg_cfg, app_cfg);     
  
  // Load point clouds from file
  DP ref = DP::load(reg_cfg.cloud_name_A);
  DP data = DP::load(reg_cfg.cloud_name_B);

  //=================================
  // TRANSFORM 3D CLOUD
  //=================================

  app->registr_->getICPTransform(data, ref);
  
  PM::TransformationParameters T = app->registr_->getTransform();
  cout << "3D Transformation:" << endl << T << endl;
  
  return 0;
}

// Make sure that the command arguments make sense
int validateArgs(const int argc, const char *argv[], RegistrationConfig& reg_cfg)
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
      reg_cfg.configFile3D_.append(reg_cfg.homedir);
      reg_cfg.configFile3D_.append("/main-distro/software/perception/registeration/filters_config/");
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
  cerr << endl;
}