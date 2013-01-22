// LCM example program for interacting  with PCL data
// In this case it pushes the data back out to LCM in
// a different message type for which the "collections"
// renderer can view it in the LCM viewer
// mfallon aug 2012
#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>


#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/visualization.h>

#include <lcmtypes/drc_lcmtypes.h>


#include "registeration.hpp"

using namespace std;
using namespace pcl;



using namespace cv;

/////////////////////////////////////

registeration::registeration(lcm_t* publish_lcm):
      publish_lcm_(publish_lcm),
      current_poseT(0, Eigen::Isometry3d::Identity()),
      null_poseT(0, Eigen::Isometry3d::Identity()),
      local_poseT(0, Eigen::Isometry3d::Identity()) {

  current_pose_init = false;

  // camera-to-lidar tf
  // TODO read from config later:
  // directly matches the frames in gazebo
  Eigen::Quaterniond quat =euler_to_quat(0,0,M_PI/2); // ypr
  Eigen::Isometry3d pose;
  camera_to_lidar.setIdentity();
  camera_to_lidar.translation()  << 0.275, 0.0, 0.252;
  camera_to_lidar.rotate(quat);


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clf (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud = cloud_clf;
  cloud_counter =0;

  // Unpack Config:
  pc_lcm_ = new pointcloud_lcm(publish_lcm_);


  // Vis Config:
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(600,"Pose - Laser",5,0) );
  float colors_a[] ={1.0,0.0,0.0};
  vector <float> colors_v;
  colors_v.assign(colors_a,colors_a+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(601,"Cloud - Laser"         ,1,0, 600,1,colors_v));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(400,"Pose - Camera",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(401,"Cloud - Current Camera",1,1, 400,0,colors_v));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(800,"Pose - Laser Map Init",5,0) );
  float colors_b[] ={0.0,0.0,1.0};
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(801,"Cloud - Laser Map"     ,1,1, 1000,1,colors_v));

  // LCM:
  lcm_t* subscribe_lcm_ = publish_lcm_;
  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "LOCALIZE_NEW_MAP",
                             newmap_handler_aux, this);
  drc_pointcloud2_t_subscribe(subscribe_lcm_, "WIDE_STEREO_POINTS",
                             registeration::pointcloud_handler_aux, this);
  bot_core_planar_lidar_t_subscribe(subscribe_lcm_, "BASE_SCAN",
                             registeration::lidar_handler_aux, this);
  bot_core_pose_t_subscribe(subscribe_lcm_, "POSE",
      registeration::pose_handler_aux, this);


  min_inliers_ = 30;

}



void registeration::newmap_handler(const drc_localize_reinitialize_cmd_t *msg){
  // Send the final version of the previous local map
  vector <float> colors_v;
  float colors_a[3];
  colors_a[0] = vis_colors[3*(cloud_counter%num_vis_colors)];
  colors_a[1] = vis_colors[3*(cloud_counter%num_vis_colors)+1];
  colors_a[2] = vis_colors[3*(cloud_counter%num_vis_colors)+2];
  colors_v.assign(colors_a,colors_a+4*sizeof(float));
  stringstream ss;
  ss << "Cloud - Local Map " << cloud_counter;
  int map_coll_id = cloud_counter + 1000 + 1; // so that first is 1001 and null is 1000
  ptcld_cfg pcfg = ptcld_cfg(map_coll_id,    ss.str()     ,1,1, 1000,1,colors_v);
  pc_vis_->ptcld_to_lcm(pcfg, *cloud, null_poseT.utime, null_poseT.utime );

  pcl::PCDWriter writer;
  stringstream ss2;
  ss2 << "cloud" << cloud_counter << ".pcd";
  writer.write (ss2.str(), *cloud, false);

  cout << cloud_counter << " finished | " << cloud->points.size() << " points\n";

  // Start a new local map:
  cloud->width    = 0;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (0);
  cloud_counter++;
  cout << cloud_counter << " started | " << cloud->points.size() << " points\n";

  Eigen::Isometry3d local_pose = current_poseT.pose*camera_to_lidar;
  local_poseT = Isometry3dTime(msg->utime, local_pose);
  pc_vis_->pose_to_lcm_from_list(800, local_poseT);
}


void registeration::pose_handler(const bot_core_pose_t *msg){
  current_poseT.pose.setIdentity();
  current_poseT.pose.translation() << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond m;
  m  = Eigen::Quaterniond(msg->orientation[0],msg->orientation[1],msg->orientation[2],msg->orientation[3]);
  current_poseT.pose.rotate(m);
  current_poseT.utime = msg->utime;

  if (!current_pose_init){
    current_pose_init = true;

    Eigen::Isometry3d local_pose = current_poseT.pose*camera_to_lidar;
    local_poseT = Isometry3dTime(msg->utime, local_pose);
    pc_vis_->pose_to_lcm_from_list(800, local_poseT);

    //c Null Pose
    Eigen::Isometry3d lidar_pose;
    lidar_pose.setIdentity();
    null_poseT = Isometry3dTime(msg->utime, lidar_pose);
    pc_vis_->pose_to_lcm_from_list(1000, null_poseT);
//    vcfg->PublishPoseToLCMFromList(3001,null_pose,cloud_utime_);
  }
}

void registeration::lidar_handler(const bot_core_planar_lidar_t *msg){
  if (!current_pose_init){     return;  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  double maxRange = 9.9;//29.7;
  //double validBeamAngles[] ={-2.5,2.5}; // all of it both sides
  double validBeamAngles[] ={-M_PI/2,2.5}; //  nothing beyond directly down  | all returns up
  convertLidar(msg->ranges, msg->nranges, msg->rad0,
        msg->radstep, lidar_cloud, maxRange,
        validBeamAngles[0], validBeamAngles[1]);

  // 3. Transmit data to viewer at null pose:
  // 3a. Transmit Pose:
  int pose_id=msg->utime;
  int lidar_obj_collection= 600;

  // Use the current VO pose without interpolation or tf:
  //Isometry3dTime lidar_poseT = Isometry3dTime(pose_id, current_poseT.pose);

  // Transform the VO pose via the inter sensor config:
  Eigen::Isometry3d lidar_pose = current_poseT.pose*camera_to_lidar;
  Isometry3dTime lidar_poseT = Isometry3dTime(pose_id, lidar_pose);
  pc_vis_->pose_to_lcm_from_list(lidar_obj_collection, lidar_poseT);
  pc_vis_->ptcld_to_lcm_from_list(601, *lidar_cloud, pose_id, pose_id);


  Eigen::Isometry3f pose_f = Isometry_d2f(lidar_poseT.pose);
  Eigen::Quaternionf pose_quat(pose_f.rotation());
  pcl::transformPointCloud (*lidar_cloud, *lidar_cloud,
      pose_f.translation(), pose_quat); // !! modifies lidar_cloud

  (*cloud) += (*lidar_cloud);
  pc_vis_->ptcld_to_lcm_from_list(801, *cloud, null_poseT.utime, null_poseT.utime);
  pc_vis_->pose_to_lcm_from_list(1000, null_poseT); // remove this?

}


void registeration::pointcloud_handler(const drc_pointcloud2_t *msg){
  if (!current_pose_init){     return;  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pc_lcm_->unpack_pointcloud2( (const ptools_pointcloud2_t*)   msg, cloud);
  //pc_lcm_->unpack_pointcloud2(  msg, cloud);

  // 3a. Transmit Pose:
  int pose_id=msg->utime;
  int camera_obj_collection= 400;

  // Use the current VO pose without interpolation or tf:
  Isometry3dTime camera_poseT = Isometry3dTime(pose_id, current_poseT.pose);
  pc_vis_->pose_to_lcm_from_list(camera_obj_collection, camera_poseT);
  pc_vis_->ptcld_to_lcm_from_list(401, *cloud, pose_id, pose_id);
}





void get_descriptors(cv::Mat &image, cv::Mat &descriptors, std::vector<bool> &valid){
  // 1. Extract Descriptors:
  cv::BriefDescriptorExtractor extractor(32); // size of descriptor in bytes

  // Downsample
  cv::Mat image_small;
  cv::resize(image, image_small, cv::Size(60, 60));

  // Compute descriptor

  std::vector<cv::KeyPoint> keypoints;
  keypoints.push_back(cv::KeyPoint(cv::Point2f(30,30), 1.0));
  extractor.compute(image_small, keypoints, descriptors);

  valid.resize(descriptors.size().height);
  std::fill(valid.begin(), valid.end(), true);
}

void registeration::go(){
  cout << "going...\n";
  Mat image0,image0_right; // image is left
  Mat image1,image1_right; // image is left
  image0 = imread("/home/mfallon/data/drc/registeration/0/left.png",CV_LOAD_IMAGE_GRAYSCALE);
  image0_right = imread("/home/mfallon/data/drc/registeration/0/right.png",CV_LOAD_IMAGE_GRAYSCALE);

  image1 = imread("/home/mfallon/data/drc/registeration/1/left.png",CV_LOAD_IMAGE_GRAYSCALE);
  image1_right = imread("/home/mfallon/data/drc/registeration/1/right.png",CV_LOAD_IMAGE_GRAYSCALE);


  imshow("Left", image0);
  imshow("Right", image0_right);
  waitKey(0);                                          // Wait for a keystroke in the window


  get_descriptors(image0, descriptors0, valid0);
  get_descriptors(image1, descriptors1, valid1);


//  if (  ref->features().size() < min_inliers_ || target->features().size() < min_inliers_ )
//  {



}

int
main(int argc, char ** argv)
{
  lcm_t * lcm;
  lcm = lcm_create(NULL);
  registeration app(lcm);

  app.go();

//  while(1)
//    lcm_handle(lcm);

//  lcm_destroy(lcm);
  return 0;
}

