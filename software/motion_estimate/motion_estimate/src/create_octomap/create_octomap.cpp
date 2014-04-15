// Start. collect 400 scans
// exit callback
// convert to octomap, blur and save to file.
// Republish the octomap
// se-create-octomap  -s 500 -b 0.5 -r 20 -w -u
//
// min range of 2m seems the right choice: culls self-observations but doesnt cull  ground in front of the robot
#include <sstream>      // std::stringstream
#include <boost/thread.hpp>

#include "convert_octomap.hpp"
#include "cloud_accumulate.hpp"

#include <lcmtypes/drc/utime_t.hpp>
#include <lcmtypes/drc/system_status_t.hpp>

#include <ConciseArgs>

struct AppConfig
{
  bool accum_at_launch;
  double repeat_period;    
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, ConvertOctomapConfig co_cfg_,
        CloudAccumulateConfig ca_cfg_, AppConfig app_cfg);
    
    ~App(){
    }        
    
    ConvertOctomapConfig co_cfg_;
    CloudAccumulateConfig ca_cfg_;
    AppConfig app_cfg_;
    
    boost::shared_ptr<lcm::LCM> lcm_;
    CloudAccumulate* accu_;
    ConvertOctomap* convert_;
    
    bool do_accum_; // true if we should accumulate a lidar point cloud (or continue to)
    bool do_convert_cloud_; // true if a new pt cloud is ready to be converted to octomap
    bool do_republish_; // true if an octree is ready   
    
    int64_t last_publish_utime_;
    
    void startMapHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  drc::utime_t* msg);   
    
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    
    void sendSystemStatus(std::string message);

private:
    
};

App::App(boost::shared_ptr< lcm::LCM >& lcm_, ConvertOctomapConfig co_cfg_,
         CloudAccumulateConfig ca_cfg_, AppConfig app_cfg_) : lcm_(lcm_), 
         co_cfg_(co_cfg_), ca_cfg_(ca_cfg_),
         app_cfg_(app_cfg_){
  convert_ = new ConvertOctomap(lcm_, co_cfg_);
  accu_ = new CloudAccumulate(lcm_, ca_cfg_);
  
  do_accum_ = app_cfg_.accum_at_launch;
  if (app_cfg_.accum_at_launch){
    std::cout << "Accumulating map at launch\n";
  }else{
    std::cout << "Waiting for message before starting map\n";
  }
  
  do_convert_cloud_ = false;
  do_republish_ = false;
  last_publish_utime_ =0;
}


void App::startMapHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg){
  std::cout << "Start map message received\n";
  do_accum_ = true;
  accu_->clearCloud();
  // TODO: empty the currently held point cloud
}


void App::sendSystemStatus(std::string message){
  drc::system_status_t status;
  status.system = drc::system_status_t::MOTION_ESTIMATION;
  status.importance = drc::system_status_t::VERY_IMPORTANT;
  status.frequency = drc::system_status_t::LOW_FREQUENCY;
  status.value = message;
  lcm_->publish("SYSTEM_STATUS", &status);
}


void App::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  if ( do_accum_ ){
    
     if ( accu_->getCounter() % 200 == 0){
      std::stringstream message;
      message << accu_->getCounter() <<  " of " << ca_cfg_.batch_size << " scans collected";
      sendSystemStatus( message.str() ); 
      std::cout << message.str() << "\n";
    }
    
    
    accu_->processLidar(msg);
    
    
    
    if ( accu_->getFinished()  ){//finished_accumating?
      do_convert_cloud_ = true;
      do_accum_ = false;
    }
  }
}



////////// Threads //////////////////////
void processThread(App& app) { 
  cout << "Started processThread\n";  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
  while (1==1) {
    usleep(1e6); // sleep for 1sec, check for new msgs and process
  
    if ( app.do_convert_cloud_ ) {
      cloud = app.accu_->getCloud();
      // LCM collections can only handle about 200k points:
      sub_cloud->points.clear();
      for (size_t i=0 ; i < cloud->points.size() ; i=i+20){
        sub_cloud->points.push_back( cloud->points[i] );
        if (sub_cloud->points.size() > 100000){ // reasonable upper limit in size
          break;
        }
      }
      sub_cloud->width = sub_cloud->points.size();
      sub_cloud->height = 1;  
      
      std::stringstream message;
      message << "Processing cloud with " << cloud->width * cloud->height
              << " points" ;
      app.sendSystemStatus( message.str() ); 
      std::cout << message.str() << "\n";      
      app.convert_->doWork(cloud);
      
      std::stringstream message2;
      message2 << "Finished processing. Click \"Use New Map\" to enable";
      app.sendSystemStatus( message2.str() ); 
      std::cout << message2.str() << "\n";      
      
      
      app.do_republish_ = true;
      app.do_convert_cloud_ =false;
    }

    if (app.do_republish_){
      if ( bot_timestamp_now() > app.last_publish_utime_ + 1e6 * app.app_cfg_.repeat_period ){
        //std::cout << "Republishing unblurred octomap and point cloud\n";
        fprintf(stderr, "r");
        app.convert_->publishOctree( app.convert_->getTree(),"OCTOMAP");
        app.accu_->publishCloud(sub_cloud);
        app.last_publish_utime_ = bot_timestamp_now();
      }    
    }
  
  }
  cout << "Finished processThread\n";    
}

void commsThread(App& app) { 
  cout << "Started commsThread\n";
  while( 0==app.lcm_->handle() );// &&  (! app.accu_->getFinished()) );
  cout << "Finished commsThread\n";  
}


int main(int argc, char ** argv) {
  ConvertOctomapConfig co_cfg;
  co_cfg.blur_sigma = 0.1; // default was .5
  co_cfg.blur_map = true;
  CloudAccumulateConfig ca_cfg;
  ca_cfg.lidar_channel ="SCAN";
  ca_cfg.batch_size = 1500;
  ca_cfg.min_range = 2.0; // remove all the short range points
  ca_cfg.max_range = 30.0;
  AppConfig app_cfg;
  app_cfg.accum_at_launch = false;
  app_cfg.repeat_period = 20; // default was -1 
  
  std::stringstream s;
  s <<  getDataPath() <<   "/octomap.pcd" ;
  std::string pcd_filename = s.str();
  int input = 0; // 0 = lcm | 1 = file
  
  ConciseArgs opt(argc, (char**)argv);
  //
  // opt.add(co_cfg.blur_map, "u", "blur_map","Blur map here");
  opt.add(co_cfg.blur_sigma, "b", "blur_sigma","Radius of the blur kernel");
  //
  opt.add(ca_cfg.lidar_channel, "l", "lidar_channel","lidar_channel");
  opt.add(ca_cfg.batch_size, "s", "batch_size","Size of the batch of scans");
  opt.add(ca_cfg.min_range, "m", "min_range","Min Range to use");
  //
  opt.add(pcd_filename, "f", "pcd_filename","Process this PCD file");    
  opt.add(input, "i", "input","Input mode: 0=lcm 1=file 2=republish pcd only");    
  opt.add(app_cfg.accum_at_launch, "a", "accum_at_launch","Start building the octomap at launch");    
  opt.add(app_cfg.repeat_period, "r", "repeat_period","Repeat period of republishes [sec]");
  opt.parse();  
  
  std::cout << "Accumulating: " << ca_cfg.batch_size << " scans\n";
  std::cout << "Blur sigma: " << co_cfg.blur_sigma << "\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App* app= new App(lcm, co_cfg, ca_cfg, app_cfg);
  
  boost::thread_group thread_group;
  
  lcm->subscribe( ca_cfg.lidar_channel, &App::lidarHandler, app);
  lcm->subscribe( "STATE_EST_START_NEW_MAP", &App::startMapHandler, app);

  thread_group.create_thread(boost::bind(processThread, boost::ref( *app)));
  thread_group.create_thread(boost::bind(commsThread, boost::ref( *app)));
  thread_group.join_all();      
  
  return 0;
}



/*
      }else{
        cout << "Reading point cloud from file" << endl << "============================" << endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_filename, *cloud) == -1){ //* load the file
          std::cout << "Couldn't read pcd file\n";
          exit(-1);
        }      
      }
      
      
      if (input == 2){
        std::cout << sub_cloud->points.size() << " sub_cloud\n";
        std::cout << cloud->points.size() << " cloud\n";
        std::cout << "Republishing pcd cloud only\n";
        app.accu_->publishCloud(sub_cloud);
        return 0;
      }      
*/