// collect 400 scans, save point cloud, exit at end 

#include <sstream>      // std::stringstream

#include "cloud_accumulate.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <ConciseArgs>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "clouds_io_utils.h"

const char *homedir;

struct AppConfig
{
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, 
        CloudAccumulateConfig ca_cfg_, AppConfig app_cfg);
    
    ~App(){
    }        
    
    CloudAccumulateConfig ca_cfg_;
    AppConfig app_cfg_;
    
    boost::shared_ptr<lcm::LCM> lcm_;
    CloudAccumulate* accu_;
    
    void planarLidarHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::planar_lidar_t* msg);   

private:
    
};

App::App(boost::shared_ptr< lcm::LCM >& lcm_, 
         CloudAccumulateConfig ca_cfg_, AppConfig app_cfg_) : lcm_(lcm_), 
         ca_cfg_(ca_cfg_),
         app_cfg_(app_cfg_){
  accu_ = new CloudAccumulate(lcm_, ca_cfg_);
  std::cout << "Accumulating map at launch\n";  
}

void App::planarLidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
    
     if ( accu_->getCounter() % 200 == 0){
      std::stringstream message;
      message << accu_->getCounter() <<  " of " << ca_cfg_.batch_size << " scans collected";
      std::cout << message.str() << "\n";
    }
    
    accu_->processLidar(msg);
    
    if ( accu_->getFinished()  ){//finished_accumating?

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      cloud = accu_->getCloud();
      cloud->width = cloud->points.size();
      cloud->height = 1;

      std::stringstream message;
      message << "Processing cloud with " << cloud->points.size()
              << " points" ;
      std::cout << message.str() << "\n";      

      if(ca_cfg_.lidar_channel == "FIXED_SCAN")
      {
        // Writing to .csv file when planar pointcloud from fixed laser 
        std::stringstream csv_fname;
        csv_fname << homedir << "/logs/multisenselog__2015-11-16/tmp/scan_" << "09" << ".csv";
        std::cout << csv_fname.str() << " written\n";
        pcl::PCLPointCloud2::Ptr cloud_output (new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2 (*cloud, *cloud_output);
        savePlanarCloudCSV (csv_fname.str(), *cloud_output);
      }
      else
      {
        pcl::PCDWriter writer;
        std::stringstream pcd_fname;
        pcd_fname << homedir << "/logs/multisenselog__2015-11-16/tmp/multisense_" << "02" << ".pcd";
        std::cout << pcd_fname.str() << " written\n";
        writer.write (pcd_fname.str() , *cloud, false);  

        std::stringstream vtk_fname;
        vtk_fname << homedir << "/logs/multisenselog__2015-11-16/tmp/multisense_" << "02" << ".vtk";
        std::cout << vtk_fname.str() << " written\n";
        pcl::PCLPointCloud2::Ptr cloud_output (new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2 (*cloud, *cloud_output);
        pcl::io::saveVTKFile (vtk_fname.str(), *cloud_output);
      }

      accu_->publishCloud(cloud);

      exit(-1);
    }
}


int main(int argc, char ** argv) {
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  CloudAccumulateConfig ca_cfg;
  ca_cfg.lidar_channel ="SCAN";
  ca_cfg.batch_size = 240; // about 1 sweep
  ca_cfg.min_range = 0.0;//1.85; // remove all the short range points
  ca_cfg.max_range = 30.0;
  AppConfig app_cfg;
  
  ConciseArgs opt(argc, (char**)argv);
  opt.add(ca_cfg.lidar_channel, "l", "lidar_channel","lidar_channel");
  opt.add(ca_cfg.batch_size, "s", "batch_size","Size of the batch of scans");
  opt.add(ca_cfg.min_range, "m", "min_range","Min Range to use");
  opt.add(ca_cfg.max_range, "x", "max_range","Max Range to use");
  opt.parse();  
  
  std::cout << "Accumulating: " << ca_cfg.batch_size << " scans\n";
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App* app= new App(lcm, ca_cfg, app_cfg);
  
  std::cout << "Subscribing to planar lidar on " << ca_cfg.lidar_channel << "\n";
  lcm->subscribe( ca_cfg.lidar_channel, &App::planarLidarHandler, app);
  while( 0==lcm->handle() );
 
  return 0;
}
