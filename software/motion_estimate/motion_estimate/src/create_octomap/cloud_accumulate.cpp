#include "cloud_accumulate.hpp"

CloudAccumulate::CloudAccumulate(boost::shared_ptr<lcm::LCM> &lcm_, const CloudAccumulateConfig& ca_cfg_):
    lcm_(lcm_), ca_cfg_(ca_cfg_){
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  
  
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
  
  
  bot_core_planar_lidar_t * laser_msg_c = convertPlanarLidarCppToC(this_msg);
  
  double zeros_lin[3] = { 0 };
  double zeros_ang[3] = { 0 };
  //zeros_ang[2] = -2.5; 
  // 100 scans per rev = 2.5 sec per rev = 24 rpm = 144 deg per second = 2.5136 rad/sec
  // 3.6 deg per scan. 
  
  // 600 scans per rev = 15 sec per rev = 4rpm = 24 deg per second = 0.4189 rad/sec
  // 0.6 deg per scan.
  
  // Old Buggy mode:
  //projected_laser_scan_ = laser_create_projected_scan_from_planar_lidar_with_motion(laser_projector_,
  //  laser_msg_c, "body", zeros_ang, zeros_lin);

  projected_laser_scan_ = laser_create_projected_scan_from_planar_lidar_with_interpolation(laser_projector_,
    laser_msg_c, "body");
  
  
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
    if (( projected_laser_scan_->rawScan->ranges[i] < 30.0) && ( projected_laser_scan_->rawScan->ranges[i] > 2.0)){
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
  Eigen::Isometry3d local_to_scan_start;
  Eigen::Isometry3d local_to_scan_end;
  int64_t scan_start_utime = this_msg->utime ; // for old logs
  int64_t scan_end_utime = this_msg->utime +  1E6*3/(40*4) ; // for old logs suould be 18750 = 0.75

  // std::cout << scan_start_utime << " and " << scan_end_utime << "\n";
  
  botframes_cpp_->get_trans_with_utime( botframes_ , "SCAN","local"  , scan_start_utime, local_to_scan_start);  
  botframes_cpp_->get_trans_with_utime( botframes_ , "SCAN", "local"  , scan_end_utime, local_to_scan_end);  
  lidar_utils_.interpolateScan(this_msg->ranges, this_msg->rad0, this_msg->radstep,
          local_to_scan_start, local_to_scan_end, points);    

  scan_local->width   = points.size();
  scan_local->height   = 1;
  scan_local->points.resize (points.size());
  
  int n_valid =0;
  for (int i = 0; i < points.size(); i++) {
    if ( (this_msg->ranges[i]<30.0)  && (this_msg->ranges[i]>2.0)  ){
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


void CloudAccumulate::publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  Isometry3dTime null_T = Isometry3dTime(1, Eigen::Isometry3d::Identity()  );
  pc_vis_->pose_to_lcm_from_list(60010, null_T);
  pc_vis_->ptcld_to_lcm_from_list(60012, *cloud, 1,1);    
}





void CloudAccumulate::processLidar(const  bot_core::planar_lidar_t* msg){
  
  if (!frame_check_utils_.isLocalToScanValid(botframes_)){
    return;
  }
  
  std::shared_ptr<bot_core::planar_lidar_t>  this_msg;
  
  int use_deque =1;
  int convert_mode =2;
  if (convert_mode ==2){ // deque fails with laser_create_projected_scan_from_planar_lidar_with_interpolation
    use_deque =0;
  }
  
  if (use_deque){  
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
  
  if (convert_mode==1){  
    scan_local = convertMode1( this_msg );
  }else if (convert_mode==2){
    scan_local = convertMode2( this_msg );
  }else{
    scan_local = convertMode3( this_msg );
  }
    
  // Accumulate
  *combined_cloud_ += *scan_local;
  
  
  
  counter_++;  
  if (counter_ > ca_cfg_.batch_size){
    std::cout << "Finished Collecting: " << this_msg->utime << "\n";
    finished_ = true;
  }

}


