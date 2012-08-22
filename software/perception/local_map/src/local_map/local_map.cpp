// LCM example program for interacting  with PCL data
// In this case it pushes the data back out to LCM in
// a different message type for which the "collections"
// renderer can view it in the LCM viewer
// mfallon aug 2012
#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/visualization.h>

#include <lcmtypes/drc_lcmtypes.h>


#include "local_map.hpp"

using namespace std;
using namespace pcl;

lcm_t * lcm;


local_map::local_map(lcm_t* publish_lcm):
      publish_lcm_(publish_lcm),current_pose(0, Eigen::Isometry3d::Identity())  {

  current_pose.pose.setIdentity();
  current_pose_init = false;

  // camera-to-lidar tf
  // TODO read from config later:
  // directly matches the frames in gazebo
  Eigen::Quaterniond quat =euler_to_quat(0,0,M_PI/2); // ypr
  Eigen::Isometry3d pose;
  camera_to_lidar.setIdentity();
  camera_to_lidar.translation()  << 0.275, 0.0, 0.252;
  camera_to_lidar.rotate(quat);

  // Unpack Config:
  pc_lcm_ = new pointcloud_lcm(publish_lcm_);


  // Vis Config:
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(600,"Laser Pose",5,0) );
  float colors_a[] ={1.0,0.0,0.0};
  vector <float> colors_v;
  colors_v.assign(colors_a,colors_a+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(601,"Laser"         ,1,0, 600,1,colors_v));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(400,"Camera Pose",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(401,"Current Camera",1,1, 400,0,colors_v));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(800,"Laser Map Pose",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(801,"Laser Map"     ,1,0, 800,1,colors_v));

  // LCM:
  lcm_t* subscribe_lcm_ = publish_lcm_;
  drc_pointcloud2_t_subscribe(subscribe_lcm_, "WIDE_STEREO_POINTS",
                             local_map::pointcloud_handler_aux, this);
  bot_core_planar_lidar_t_subscribe(subscribe_lcm_, "BASE_SCAN",
                             local_map::lidar_handler_aux, this);
  bot_core_pose_t_subscribe(subscribe_lcm_, "POSE",
      local_map::pose_handler_aux, this);
}

void local_map::pose_handler(const bot_core_pose_t *msg){
  current_pose.pose.setIdentity();
  current_pose.pose.translation() << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond m;
  m  = Eigen::Quaterniond(msg->orientation[0],msg->orientation[1],msg->orientation[2],msg->orientation[3]);
  current_pose.pose.rotate(m);
  current_pose.utime = msg->utime;

  if (!current_pose_init){     current_pose_init = true;  }
}

void local_map::lidar_handler(const bot_core_planar_lidar_t *msg){
  if (!current_pose_init){     return;  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  double maxRange = 9.9;//29.7;
  double validBeamAngles[] ={-2.1,2.1};
  convertLidar(msg->ranges, msg->nranges, msg->rad0,
        msg->radstep, cloud, maxRange,
        validBeamAngles[0], validBeamAngles[1]);

  // 3. Transmit data to viewer at null pose:
  // 3a. Transmit Pose:
  int pose_id=msg->utime;
  int lidar_obj_collection= 600;

  // Use the current VO pose without interpolation or tf:
  //Isometry3dTime lidar_poseT = Isometry3dTime(pose_id, current_pose.pose);

  // Transform the VO pose via the inter sensor config:
  Eigen::Isometry3d lidar_pose = current_pose.pose*camera_to_lidar;
  Isometry3dTime lidar_poseT = Isometry3dTime(pose_id, lidar_pose);
  pc_vis_->pose_to_lcm_from_list(lidar_obj_collection, lidar_poseT);
  pc_vis_->ptcld_to_lcm_from_list(601, *cloud, pose_id, pose_id);
}


void local_map::pointcloud_handler(const drc_pointcloud2_t *msg){
  if (!current_pose_init){     return;  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  unpack_pointcloud2(msg, cloud);

  // 3a. Transmit Pose:
  int pose_id=msg->utime;
  int camera_obj_collection= 400;

  // Use the current VO pose without interpolation or tf:
  Isometry3dTime camera_poseT = Isometry3dTime(pose_id, current_pose.pose);
  pc_vis_->pose_to_lcm_from_list(camera_obj_collection, camera_poseT);
  pc_vis_->ptcld_to_lcm_from_list(401, *cloud, pose_id, pose_id);
}

int
main(int argc, char ** argv)
{
  lcm = lcm_create(NULL);
  local_map app(lcm);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}

