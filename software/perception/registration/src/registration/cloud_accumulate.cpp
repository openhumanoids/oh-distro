#include "cloud_accumulate.hpp"
#include <iomanip>      // std::setprecision
#include <pronto_utils/conversions_lcm.hpp>

CloudAccumulate::CloudAccumulate(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_):
    lcm_(lcm_), ca_cfg_(ca_cfg_){
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  
  
  bool reset =0;
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose - Laser",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60001,"Cloud - Laser"         ,1,reset, 60000,1, {0.0, 0.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60010,"Pose - Null",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60011,"Cloud (scan-by-scan) - Null"         ,1,reset, 60010,1, {0.0, 1.0, 0.0} ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60012,"Cloud (full sweep) - Null"         ,1,1, 60010,1, {1.0, 0.0, 0.0} ));
  
  counter_ =0;  
  verbose_=3; // 1 important, 2 useful 3, lots
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  combined_cloud_ = cloud_ptr;    
  
  finished_ = false;
  
  const char * laser_name;
  if(ca_cfg_.lidar_channel == "FIXED_SCAN")
    laser_name = ca_cfg_.lidar_channel.c_str();
  else
    laser_name = "laser";

  laser_projector_ = laser_projector_new(botparam_, botframes_, laser_name, 1); //TODO: "laser" name should be a param
}


bot_core_planar_lidar_t * convertPlanarLidarCppToC(std::shared_ptr<bot_core::planar_lidar_t> this_msg){

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
  return laser_msg_c;
}


int get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        Eigen::Isometry3d & mat){
  int status;
  double matx[16];
  status = bot_frames_get_trans_mat_4x4_with_utime( bot_frames, from_frame,  to_frame, utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i,j) = matx[i*4+j];
    }
  }  
  return status;
}


void CloudAccumulate::publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  Eigen::Isometry3d local_to_fixscan;
  get_trans_with_utime( botframes_ , ca_cfg_.lidar_channel.c_str(), "local"  , 1, local_to_fixscan);
  
  Isometry3dTime local_to_fixscan_T = Isometry3dTime(1, local_to_fixscan );
  Isometry3dTime null_T = Isometry3dTime(1, Eigen::Isometry3d::Identity()  );
  pc_vis_->pose_to_lcm_from_list(60000, local_to_fixscan_T);
  pc_vis_->ptcld_to_lcm_from_list(60001, *cloud, 1,1);
  pc_vis_->pose_to_lcm_from_list(60010, null_T);
  pc_vis_->ptcld_to_lcm_from_list(60012, *cloud, 1,1);
}



void transformPointCloud(pronto::PointCloud &cloud_in, pronto::PointCloud &cloud_out, Eigen::Affine3f transform){
  int npts = cloud_in.points.size();
  cloud_out.points.resize(npts);
  for(int j=0; j<npts; j++) {
    Eigen::Vector3f pt_out = transform* Eigen::Vector3f(cloud_in.points[j].x, cloud_in.points[j].y, cloud_in.points[j].z);
    cloud_out.points[j].x = pt_out(0);
    cloud_out.points[j].y = pt_out(1);
    cloud_out.points[j].z = pt_out(2);
    cloud_out.points[j].r = cloud_in.points[j].r;
    cloud_out.points[j].g = cloud_in.points[j].g;
    cloud_out.points[j].b = cloud_in.points[j].b;
  }
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr  CloudAccumulate::convertPlanarScanToCloud(std::shared_ptr<bot_core::planar_lidar_t> this_msg){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_local (new pcl::PointCloud<pcl::PointXYZRGB> ());
  
  // 1. Convert the Lidar scan into a libbot set of points
  bot_core_planar_lidar_t * laser_msg_c = convertPlanarLidarCppToC(this_msg);
  // 100 scans per rev = 2.5 sec per rev = 24 rpm = 144 deg per second = 2.5136 rad/sec, 3.6 deg per scan.
  // 600 scans per rev = 15 sec per rev = 4rpm = 24 deg per second = 0.4189 rad/sec, 0.6 deg per scan.
  projected_laser_scan_ = laser_create_projected_scan_from_planar_lidar_with_interpolation(laser_projector_,
    laser_msg_c, "body");
  if (projected_laser_scan_ == NULL){
    std::cout << "projection failed\n";
    return scan_local;
  }

  // 2. Convert set of points into a point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_body (new pcl::PointCloud<pcl::PointXYZRGB> ());
  scan_body->points.resize (projected_laser_scan_->npoints);
  int n_valid =0;
  for (int i = 0; i < projected_laser_scan_->npoints; i++) {
    if (( projected_laser_scan_->rawScan->ranges[i] < ca_cfg_.max_range) && ( projected_laser_scan_->rawScan->ranges[i] > ca_cfg_.min_range)){
      scan_body->points[n_valid].x = projected_laser_scan_->points[i].x;
      scan_body->points[n_valid].y = projected_laser_scan_->points[i].y;
      scan_body->points[n_valid].z = projected_laser_scan_->points[i].z;
      n_valid++;
    }
  }  
  scan_body->points.resize (n_valid);  
  bot_core_planar_lidar_t_destroy(laser_msg_c);
  laser_destroy_projected_scan(projected_laser_scan_);  
  
  // 3. Visualize the scan:
  Eigen::Isometry3d body_to_local;
  get_trans_with_utime( botframes_ , "body", "local"  , this_msg->utime, body_to_local);
  
  // 4. Project the scan into local frame:
  transformPointCloud(*scan_body, *scan_local, Eigen::Affine3f ( body_to_local.cast<float>() ) );
  Isometry3dTime null_T = Isometry3dTime(counter_, Eigen::Isometry3d::Identity()  );
  if (verbose_>=2) pc_vis_->pose_to_lcm_from_list(60010, null_T);
  if (verbose_>=2) pc_vis_->ptcld_to_lcm_from_list(60011, *scan_local, counter_, counter_);
  
  return scan_local;
}


void CloudAccumulate::processLidar(const  bot_core::planar_lidar_t* msg){
  
//  if (!frame_check_tools_.isLocalToScanValid(botframes_)){
//    return;
//  }
  
  // Convert Scan to local frame:
  std::shared_ptr<bot_core::planar_lidar_t>  this_msg;
  this_msg = std::shared_ptr<bot_core::planar_lidar_t>(new bot_core::planar_lidar_t(*msg));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_local (new pcl::PointCloud<pcl::PointXYZRGB> ());
  scan_local = convertPlanarScanToCloud( this_msg );
    
  // Accumulate
  combined_cloud_->points.insert(combined_cloud_->points.end(), scan_local->points.begin(), scan_local->points.end());
  
  counter_++;  
  if (counter_ >= ca_cfg_.batch_size){
    std::cout << "Finished Collecting: " << this_msg->utime << "\n";
    finished_ = true;
  }

}
