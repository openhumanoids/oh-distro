// Start. collect 400 scans
// exit callback
// convert to octomap, blur and save to file.
// Republish the octomap
// se-create-octomap  -s 500 -b 0.5 -r 20 -w -u
//
// min range of 2m seems the right choice: culls self-observations but doesnt cull  ground in front of the robot
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <path_util/path_util.h>
#include <lcmtypes/bot_core.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp>
#include <pointcloud_tools/pointcloud_lcm.hpp> // decode perception lcm messages
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <ConciseArgs>

#include "visualization/collections.hpp"
#include "convert_octomap.hpp"

////////////////////////////////////////
struct CloudAccumulateConfig
{
    int batch_size;
    std::string lidar_channel;
    double max_range;
    double min_range;
};

class CloudAccumulate{
  public:
    CloudAccumulate(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_);
    
    ~CloudAccumulate(){
    }    
    
    bool getFinished(){ return finished_; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){ return combined_cloud_; }
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CloudAccumulateConfig& ca_cfg_;
    
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    
    pointcloud_vis* pc_vis_ ;
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   
    
    int counter_; // used for terminal feedback
    int verbose_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud_;  
    
    bool finished_;
};


CloudAccumulate::CloudAccumulate(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_):
    lcm_(lcm_), ca_cfg_(ca_cfg_){
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  lcm_->subscribe( ca_cfg_.lidar_channel  ,&CloudAccumulate::lidarHandler,this);
  
  bool reset =0;
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose - Laser",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60001,"Cloud - Laser"         ,1,reset, 60000,0, {0.0, 0.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60010,"Pose - Null",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60011,"Cloud (scan-by-scan) - Null"         ,1,reset, 60010,0, {0.0, 0.0, 1.0} ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60012,"Cloud (full sweep) - Null"         ,1,1, 60010,0, {0.0, 0.0, 1.0} ));
  
  counter_ =0;  
  verbose_=3; // 1 important, 2 useful 3, lots
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  combined_cloud_ = cloud_ptr;    
  
  finished_ = false;
}

void CloudAccumulate::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_laser (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
  // 1. Convert scan into simple XY point cloud:  
  double validBeamAngles[] ={-10,10}; 
  convertLidar(msg->ranges, msg->nranges, msg->rad0,
      msg->radstep, scan_laser, ca_cfg_.min_range, ca_cfg_.max_range,
      validBeamAngles[0], validBeamAngles[1]);  
  
  // 4. Visualize the scan:
  Eigen::Isometry3d scan_to_local;
  botframes_cpp_->get_trans_with_utime( botframes_ , "SCAN", "local"  , msg->utime, scan_to_local);
  
  Isometry3dTime scan_to_local_T = Isometry3dTime(counter_, scan_to_local);
  if (verbose_>=3) pc_vis_->pose_to_lcm_from_list(60000, scan_to_local_T);
  if (verbose_>=3) pc_vis_->ptcld_to_lcm_from_list(60001, *scan_laser, counter_, counter_);  
  
  
  /////////////////
  // 2. Project the scan into local frame:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_local (new pcl::PointCloud<pcl::PointXYZRGB> ());
  Eigen::Isometry3f scan_to_local_f= scan_to_local.cast<float>();
  pcl::transformPointCloud (*scan_laser, *scan_local,
      scan_to_local_f.translation(), Eigen::Quaternionf(scan_to_local_f.rotation())  );    
  Isometry3dTime null_T = Isometry3dTime(counter_, Eigen::Isometry3d::Identity()  );
  if (verbose_>=2) pc_vis_->pose_to_lcm_from_list(60010, null_T);
  if (verbose_>=2) pc_vis_->ptcld_to_lcm_from_list(60011, *scan_local, counter_, counter_);    

  // Accumulate
  *combined_cloud_ += *scan_local;
  
  counter_++;  
  if (counter_ > ca_cfg_.batch_size){
    // too much data for a single message:
    //pc_vis_->ptcld_to_lcm_from_list(60012, *combined_cloud_, counter_, counter_);    
    
    std::stringstream s;
    s <<  getDataPath() <<   "/octomap.pcd" ;
    printf("Saving original point cloud to: %s\n", s.str().c_str());
    pcl::PCDWriter writer;
    writer.write (s.str(), *combined_cloud_, true); // binary =true
    cout << "finished writing "<< combined_cloud_->points.size() <<" points to:\n" << s.str() <<"\n";
    
    
    //stringstream ss2;
    //ss2 << "/tmp/sweep_cloud_"  << msg->utime << ".pcd";
    //
    //std::cout << "Writing ptcldToOctomapLogFile\n";
    //pc_vis_->ptcldToOctomapLogFile(*combined_cloud_, "/home/mfallon/Desktop/test.octolog");
    //writer.write ("/home/mfallon/Desktop/test.pcd", *combined_cloud_, true); // binary =true
    
    cout << "Finished Collecting: " << msg->utime << "\n";
    //  vs::reset_collections_t reset;
    //  lcm_->publish("RESET_COLLECTIONS", &reset);    
    finished_ = true;
  }

}




int main(int argc, char ** argv) {
  ConvertOctomapConfig co_cfg;
  co_cfg.blur_sigma = 0.5; // default was .5
  co_cfg.repeat_period = -1; // default was -1 
  co_cfg.write_output = false;
  co_cfg.blur_map = false; // used to do it here, but now do it in application
  CloudAccumulateConfig ca_cfg;
  ca_cfg.lidar_channel ="SCAN";
  ca_cfg.batch_size = 500;
  ca_cfg.min_range = 2.0; // remove all the short range points
  ca_cfg.max_range = 30.0;
 
  std::string pcd_filename = "/home/mfallon/data/atlas/2014-01-21-vicon-walking/octomap/working_version/example_sweep_cloud_400scans.pcd";
  int input = 0; // 0 = lcm | 1 = file
  
  ConciseArgs opt(argc, (char**)argv);
  //
  opt.add(co_cfg.blur_map, "u", "blur_map","Blur map here");
  opt.add(co_cfg.blur_sigma, "b", "blur_sigma","Radius of the blur kernel");
  opt.add(co_cfg.repeat_period, "r", "repeat_period","Repeat period of republishes [sec]");
  opt.add(co_cfg.write_output, "w", "write_output","Write output octomaps (bt, bt_blurred");
  //
  opt.add(ca_cfg.lidar_channel, "l", "lidar_channel","lidar_channel");
  opt.add(ca_cfg.batch_size, "s", "batch_size","Size of the batch of scans");
  opt.add(ca_cfg.min_range, "m", "min_range","Min Range to use");
  //
  opt.add(pcd_filename, "f", "pcd_filename","Process this PCD file");    
  opt.add(input, "i", "input","Input mode: 0=lcm 1=file");    
  opt.parse();  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  ConvertOctomap convert(lcm, co_cfg);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  if (input==0){
    CloudAccumulate accu(lcm, ca_cfg);
    cout << "Listening for LIDAR from LCM" << endl << "============================" << endl;
    while( (0==lcm->handle()) &&  (!accu.getFinished()) );
    
    std::cout <<" finished\n";
    cloud = accu.getCloud();
    
  }else{
    cout << "Reading point cloud from file" << endl << "============================" << endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_filename, *cloud) == -1){ //* load the file
      std::cout << "Couldn't read pcd file\n";
      exit(-1);
    }      
  }


  convert.doWork(cloud);
  
  
  return 0;
}
