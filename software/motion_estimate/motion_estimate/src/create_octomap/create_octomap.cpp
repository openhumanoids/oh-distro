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
#include <drc_utils/frame_check_utils.hpp>
#include <drc_utils/LidarUtils.hpp>

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
    
    void publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CloudAccumulateConfig& ca_cfg_;
    
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    
    pointcloud_vis* pc_vis_ ;
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;  
    FrameCheckUtils frame_check_utils_;
    
    drc::LidarUtils lidar_utils_;
    
    int counter_; // used for terminal feedback
    int verbose_;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud_;  
    
    bool finished_;
    
    std::deque<std::shared_ptr<bot_core::planar_lidar_t> > laser_queue_;
    
    Laser_projector * laser_projector_;
    laser_projected_scan * projected_laser_scan_;  
    
    Eigen::Isometry3d body_to_scan_last_;
    int64_t body_to_scan_last_utime_;
    
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertMode1(std::shared_ptr<bot_core::planar_lidar_t> this_msg);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertMode2(std::shared_ptr<bot_core::planar_lidar_t> this_msg);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertMode3(std::shared_ptr<bot_core::planar_lidar_t> this_msg);
    
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
  
  
  laser_projector_ = laser_projector_new(botparam_, botframes_, "laser", 1); //TODO: laser name should be a param
  std::cout << laser_projector_->max_range << "\n";
  std::cout << laser_projector_->laser_frequency << "\n";
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr  CloudAccumulate::convertMode1(std::shared_ptr<bot_core::planar_lidar_t> this_msg){
  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_laser (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_local (new pcl::PointCloud<pcl::PointXYZRGB> ());
    
    // 1. Convert scan into simple XY point cloud:  
    double validBeamAngles[] ={-10,10}; 
    convertLidar(this_msg->ranges, this_msg->nranges, this_msg->rad0,
        this_msg->radstep, scan_laser, ca_cfg_.min_range, ca_cfg_.max_range,
        validBeamAngles[0], validBeamAngles[1]);  
    
    // 4. Visualize the scan:
    Eigen::Isometry3d scan_to_local;
    botframes_cpp_->get_trans_with_utime( botframes_ , "SCAN", "local"  , this_msg->utime, scan_to_local);
    
    Isometry3dTime scan_to_local_T = Isometry3dTime(counter_, scan_to_local);
    if (verbose_>=3) pc_vis_->pose_to_lcm_from_list(60000, scan_to_local_T);
    if (verbose_>=3) pc_vis_->ptcld_to_lcm_from_list(60001, *scan_laser, counter_, counter_);  
    
    
    /////////////////
    // 2. Project the scan into local frame:
    Eigen::Isometry3f scan_to_local_f= scan_to_local.cast<float>();
    pcl::transformPointCloud (*scan_laser, *scan_local,
        scan_to_local_f.translation(), Eigen::Quaternionf(scan_to_local_f.rotation())  );    
    Isometry3dTime null_T = Isometry3dTime(counter_, Eigen::Isometry3d::Identity()  );
    if (verbose_>=2) pc_vis_->pose_to_lcm_from_list(60010, null_T);
    if (verbose_>=2) pc_vis_->ptcld_to_lcm_from_list(60011, *scan_local, counter_, counter_);      
  
    return scan_local;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  CloudAccumulate::convertMode2(std::shared_ptr<bot_core::planar_lidar_t> this_msg){
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_local (new pcl::PointCloud<pcl::PointXYZRGB> ());
    
  ////////////////////
  ////////////////////
  ////////////////////
  Eigen::Isometry3d body_to_scan;
  botframes_cpp_->get_trans_with_utime( botframes_ , "body", "SCAN"  , this_msg->utime, body_to_scan);  
  Eigen::Vector3d t(body_to_scan.translation());
  Eigen::Quaterniond r(body_to_scan.rotation());
  double rpy[3];
  quat_to_euler(r, rpy[0], rpy[1], rpy[2]);
  

  Eigen::Vector3d lin_last(body_to_scan_last_.translation());
  Eigen::Quaterniond r_last(body_to_scan_last_.rotation());
  double rpy_last[3];
  quat_to_euler(r_last, rpy_last[0], rpy_last[1], rpy_last[2]);

  double rpy_diff[3];
  rpy_diff[0] = rpy[0] - rpy_last[0];
  rpy_diff[1] = rpy[1] - rpy_last[1];
  rpy_diff[2] = rpy[2] - rpy_last[2];

  double elapsed_time = (this_msg->utime - body_to_scan_last_utime_)*1E-6;
  double rpy_rate[3];
  rpy_rate[0] = rpy_diff[0]/ elapsed_time;
  rpy_rate[1] = rpy_diff[1]/ elapsed_time;
  rpy_rate[2] = rpy_diff[2]/ elapsed_time;  
  
  
  
  Eigen::Isometry3d tf_diff =  body_to_scan_last_.inverse()  * body_to_scan  ; // this works fine
//  Eigen::Isometry3d tf_diff =   ( body_to_scan.inverse()  *  body_to_scan_last_  ).inverse();
  Eigen::Vector3d lin_diff_tf(tf_diff.translation());
  Eigen::Quaterniond r_diff_tf(tf_diff.rotation());
  double rpy_last_tf[3];
  quat_to_euler(r_diff_tf, rpy_last_tf[0], rpy_last_tf[1], rpy_last_tf[2]);
  
  double rpy_rate_tf[3];
  rpy_rate_tf[0] = rpy_last_tf[0]/ elapsed_time;
  rpy_rate_tf[1] = rpy_last_tf[1]/ elapsed_time;
  rpy_rate_tf[2] = rpy_last_tf[2]/ elapsed_time;  
  
  double lin_rate_tf[3];
  lin_rate_tf[0] = lin_diff_tf[0]/ elapsed_time;
  lin_rate_tf[1] = lin_diff_tf[1]/ elapsed_time;
  lin_rate_tf[2] = lin_diff_tf[2]/ elapsed_time;
  
  if (1==0){
    std::cout << std::setprecision(5) << std::fixed 
              << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << " | " 
              << rpy_last[0] << ", " << rpy_last[1] << ", " << rpy_last[2] << " | "
              << rpy_diff[0] << ", " << rpy_diff[1] << ", " << rpy_diff[2] << " | "
              << rpy_rate[0] << ", " << rpy_rate[1] << ", " << rpy_rate[2] << " | "
              << rpy_rate_tf[0] << ", " << rpy_rate_tf[1] << ", " << rpy_rate_tf[2] << "\n";            

    std::cout << std::setprecision(5) << std::fixed 
              << elapsed_time << " | "
              << lin_diff_tf[0] << ", " << lin_diff_tf[1] << ", " << lin_diff_tf[2] << " | "
              << lin_rate_tf[0] << ", " << lin_rate_tf[1] << ", " << lin_rate_tf[2] << "\n";            
  }  

  body_to_scan_last_ =  body_to_scan;
  body_to_scan_last_utime_ = this_msg->utime;  
  
  
  
  bot_core_planar_lidar_t * laser_msg_c = new bot_core_planar_lidar_t;
  laser_msg_c->intensities = new float[this_msg->nintensities];
  laser_msg_c->nintensities = this_msg->nintensities;
  memcpy(laser_msg_c->intensities, &this_msg->intensities[0], this_msg->nintensities * sizeof(float));

  laser_msg_c->ranges = new float[this_msg->nranges];
  laser_msg_c->nranges = this_msg->nranges;
  memcpy(laser_msg_c->ranges, &this_msg->ranges[0], this_msg->nranges * sizeof(float));

  laser_msg_c->rad0 = this_msg->rad0;
  laser_msg_c->radstep = this_msg->radstep;
  laser_msg_c->utime = this_msg->utime;  
  
  double zeros_lin[3] = { 0 };
  double zeros_ang[3] = { 0 };
  //zeros_ang[2] = -2.5; 
  // 100 scans per rev = 2.5 sec per rev = 24 rpm = 144 deg per second = 2.5136 rad/sec
  // 3.6 deg per scan. 
  
  // 600 scans per rev = 15 sec per rev = 4rpm = 24 deg per second = 0.4189 rad/sec
  // 0.6 deg per scan.
  
  
  
  projected_laser_scan_ = laser_create_projected_scan_from_planar_lidar_with_motion(laser_projector_,
    laser_msg_c, "body", zeros_ang, zeros_lin);
  //projected_laser_scan_ = laser_create_projected_scan_from_planar_lidar_with_motion(laser_projector_,
  //   laser_msg_c, "body", rpy_rate_tf, lin_rate_tf);
  if (projected_laser_scan_ == NULL){
    std::cout << "projection failed\n";
    return scan_local;
  }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_body (new pcl::PointCloud<pcl::PointXYZRGB> ());
  scan_body->width   = projected_laser_scan_->npoints;
  scan_body->height   = 1;
  scan_body->points.resize (projected_laser_scan_->npoints);
  
  int n_valid =0;
  for (int i = 0; i < projected_laser_scan_->npoints; i++) {
    // std::cout << i << " " << projected_laser_scan_->points[i].x << "\n";
    // std::cout << i << " " << projected_laser_scan_->points[i].z << "\n";   
    if (   projected_laser_scan_->rawScan->ranges[i] < 30){
      scan_body->points[n_valid].x = projected_laser_scan_->points[i].x;
      scan_body->points[n_valid].y = projected_laser_scan_->points[i].y;
      scan_body->points[n_valid].z = projected_laser_scan_->points[i].z;
      n_valid++;
    }
  }  
  // Resize outgoing cloud
  scan_body->width   = n_valid;
  scan_body->points.resize (n_valid);  
  
  bot_core_planar_lidar_t_destroy(laser_msg_c);
  laser_destroy_projected_scan(projected_laser_scan_);  
  
  
  /*
  double rpy_cal[3];
  double q_cal[4] ={    0.495782, -0.493099, -0.504123, -0.506866};
  quat_to_euler(q_cal, rpy_cal[0], rpy_cal[1], rpy_cal[2]);
    
  std::cout << std::setprecision(5) << std::fixed 
              << rpy_cal[0]*180.0/M_PI << ", " << rpy_cal[1]*180.0/M_PI << ", " << rpy_cal[2]*180.0/M_PI << " cal\n";     
  */
  

      
  // std::string body_to_scan_string = print_Isometry3d(body_to_scan);
  // std::cout << this_msg->utime << ", " << body_to_scan_string << "\n";
  
  
    // 4. Visualize the scan:
    Eigen::Isometry3d body_to_local;
    botframes_cpp_->get_trans_with_utime( botframes_ , "body", "local"  , this_msg->utime, body_to_local);  
  
    /////////////////
    // 2. Project the scan into local frame:
    Eigen::Isometry3f body_to_local_f= body_to_local.cast<float>();
    pcl::transformPointCloud (*scan_body, *scan_local,
        body_to_local_f.translation(), Eigen::Quaternionf(body_to_local_f.rotation())  );    
    Isometry3dTime null_T = Isometry3dTime(counter_, Eigen::Isometry3d::Identity()  );
    if (verbose_>=2) pc_vis_->pose_to_lcm_from_list(60010, null_T);
    if (verbose_>=2) pc_vis_->ptcld_to_lcm_from_list(60011, *scan_local, counter_, counter_);     
  
  ////////////////////
  ////////////////////
  ////////////////////
  return scan_local;
      
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr  CloudAccumulate::convertMode3(std::shared_ptr<bot_core::planar_lidar_t> this_msg){
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_local (new pcl::PointCloud<pcl::PointXYZRGB> ());


  std::vector<Eigen::Vector3f> points;
  Eigen::Isometry3d body_to_scan_start;
  Eigen::Isometry3d body_to_scan_end;
  int64_t scan_start_utime = this_msg->utime ; // for old logs
  int64_t scan_end_utime = this_msg->utime +  1E6*3/(40*4) ; // for old logs suould be 18750 = 0.75

  // std::cout << scan_start_utime << " and " << scan_end_utime << "\n";
  
  botframes_cpp_->get_trans_with_utime( botframes_ , "SCAN","local"  , scan_start_utime, body_to_scan_start);  
  botframes_cpp_->get_trans_with_utime( botframes_ , "SCAN", "local"  , scan_end_utime, body_to_scan_end);  
  
  lidar_utils_.interpolateScan(this_msg->ranges, this_msg->rad0, this_msg->radstep,
              body_to_scan_start, body_to_scan_end, points);    

  
  scan_local->width   = points.size();
  scan_local->height   = 1;
  scan_local->points.resize (points.size());
  
  int n_valid =0;
  for (int i = 0; i < points.size(); i++) {
    if (   this_msg->ranges[i] < 30){
      scan_local->points[n_valid].x = points[i](0);;
      scan_local->points[n_valid].y = points[i](1);;
      scan_local->points[n_valid].z = points[i](2);;
      n_valid++;
    }
  }  
  // Resize outgoing cloud
  scan_local->width   = n_valid;
  scan_local->points.resize (n_valid);    
  
  Isometry3dTime null_T = Isometry3dTime(counter_, Eigen::Isometry3d::Identity()  );
  if (verbose_>=2) pc_vis_->pose_to_lcm_from_list(60010, null_T);
  if (verbose_>=2) pc_vis_->ptcld_to_lcm_from_list(60011, *scan_local, counter_, counter_);   
  
  return scan_local;
}


void CloudAccumulate::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  
  if (!frame_check_utils_.isLocalToScanValid(botframes_)){
    return;
  }
  
  std::shared_ptr<bot_core::planar_lidar_t>  this_msg;
  
  if (1==1){  
    std::shared_ptr<bot_core::planar_lidar_t> data
        (new bot_core::planar_lidar_t(*msg));  
    
    laser_queue_.push_back( data );  
    if (laser_queue_.size() >= 5){
      //for (size_t i=0; i< laser_queue_.size(); i++){
      //  std::cout << i << " " << laser_queue_[i]->utime << "\n";
      //}
      
      this_msg =  laser_queue_.front();
      laser_queue_.pop_front();
      //std::cout << this_msg->utime << " is utime\n";
    }else{
      std::cout << "not processing this iteration\n";
      return; 
    }
  }else{
    this_msg = std::shared_ptr<bot_core::planar_lidar_t>(new bot_core::planar_lidar_t(*msg));      
  }
  

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_local (new pcl::PointCloud<pcl::PointXYZRGB> ());
  int mode =3;
  if (mode==1){  
    scan_local = convertMode1( this_msg );
  }else if (mode==2){
    scan_local = convertMode2( this_msg );
  }else{
    scan_local = convertMode3( this_msg );
    
  }
    
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
    //ss2 << "/tmp/sweep_cloud_"  << this_msg->utime << ".pcd";
    //
    //std::cout << "Writing ptcldToOctomapLogFile\n";
    //pc_vis_->ptcldToOctomapLogFile(*combined_cloud_, "/home/mfallon/Desktop/test.octolog");
    //writer.write ("/home/mfallon/Desktop/test.pcd", *combined_cloud_, true); // binary =true
    
    cout << "Finished Collecting: " << this_msg->utime << "\n";
    //  vs::reset_collections_t reset;
    //  lcm_->publish("RESET_COLLECTIONS", &reset);    
    finished_ = true;
  }

}


void CloudAccumulate::publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  Isometry3dTime null_T = Isometry3dTime(counter_, Eigen::Isometry3d::Identity()  );
  pc_vis_->pose_to_lcm_from_list(60010, null_T);
  pc_vis_->ptcld_to_lcm_from_list(60012, *combined_cloud_, counter_, counter_);    
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
  CloudAccumulate accu(lcm, ca_cfg);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  if (input==0){
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


  // LCM collections can only handle about 200k points:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  for (size_t i=0 ; i < 50000 ; i++){
    sub_cloud->points.push_back( cloud->points[i] );
  }
  sub_cloud->width = sub_cloud->points.size();
  sub_cloud->height = 1;  
  

  
  
  if (co_cfg.repeat_period > 0) {
    std::cout << "Republishing unblurred octomap and point cloud to LCM with period "<< co_cfg.repeat_period << " sec\n";
    while (1) {
      usleep(1e6 * co_cfg.repeat_period);
      fprintf(stderr, ".");
      convert.publishOctree( convert.getTree(),"OCTOMAP");

      accu.publishCloud(sub_cloud);
      // if (co_cfg_.blur_map) publishOctree(tree_blurred,"OCTOMAP_BLURRED");
    }
  }  


  return 0;
}
