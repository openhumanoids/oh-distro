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

#include <lcmtypes/visualization.h>
#include <deque>

#include "local_map.hpp"

using namespace std;
using namespace pcl;





/////////////////////////////////////

local_map::local_map(lcm_t* publish_lcm):
          publish_lcm_(publish_lcm),
          current_poseT(0, Eigen::Isometry3d::Identity()),
          null_poseT(0, Eigen::Isometry3d::Identity()),
          local_poseT(0, Eigen::Isometry3d::Identity()) {

  
  botparam_ = bot_param_new_from_server(publish_lcm_, 0);
  botframes_= bot_frames_get_global(publish_lcm_, botparam_);

  current_pose_init = false;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clf (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud = cloud_clf;
  cloud_counter =0;

  fill_counter =0;
  newmap_requested=0;

  // Unpack Config:
  pc_lcm_ = new pointcloud_lcm(publish_lcm_);

  // Vis Config:
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Null",5,0) );

  // LCM:
  lcm_t* subscribe_lcm_ = publish_lcm_;
  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "LOCALIZE_NEW_MAP",
      newmap_handler_aux, this);
  bot_core_planar_lidar_t_subscribe(subscribe_lcm_, "ROTATING_SCAN",
      local_map::lidar_handler_aux, this);
  
  laser_queue_ = new deque<bot_core_planar_lidar_t *> ();
}

void local_map::newmap_handler(const drc_localize_reinitialize_cmd_t *msg){
  cout << "requested newmap\n";
  newmap_requested=1;
}

void deque_to_cloud(deque<bot_core_planar_lidar_t *> * laser_queue, 
                               BotFrames* botframes, 
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &accum_cloud){
  cout <<"deque of size " << laser_queue->size() <<"to cloud\n";
  
  // Pop oldest from queue until it is empty:
  while (!laser_queue->empty()){
    bot_core_planar_lidar_t *curr_msg =  laser_queue->front();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    double maxRange = 9.9;//29.7;
    double validBeamAngles[] ={-2.5,2.5}; // all of it both sides
    convertLidar(curr_msg->ranges, curr_msg->nranges, curr_msg->rad0,
      curr_msg->radstep, lidar_cloud, maxRange,
      validBeamAngles[0], validBeamAngles[1]);
     
    BotTrans transform;
    bot_frames_get_trans_with_utime(botframes, "ROTATING_SCAN", "local",
                                          curr_msg->utime, &transform);  

    // Fixed rigid transform
    Eigen::Isometry3d local_to_lidar;
    
    Eigen::Quaterniond quat = Eigen::Quaterniond(transform.rot_quat[0], transform.rot_quat[1],transform.rot_quat[2],transform.rot_quat[3]);
    local_to_lidar.setIdentity();
    local_to_lidar.translation()  << transform.trans_vec[0], transform.trans_vec[1], transform.trans_vec[2];
    local_to_lidar.rotate(quat);

    Eigen::Isometry3f pose_f = Isometry_d2f(local_to_lidar);
    Eigen::Quaternionf pose_quat(pose_f.rotation());
    pcl::transformPointCloud (*lidar_cloud, *lidar_cloud,
        pose_f.translation(), pose_quat); // !! modifies lidar_cloud
    (*accum_cloud) += (*lidar_cloud);
            
    laser_queue->pop_front();
  }
}


void local_map::lidar_handler(const bot_core_planar_lidar_t *msg){
  laser_queue_->push_back( bot_core_planar_lidar_t_copy (msg) );
  while (laser_queue_->size() > 100){
    laser_queue_->pop_front();
  }
  
  if (newmap_requested){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accum_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    deque_to_cloud(laser_queue_, botframes_, accum_cloud);
      
    
    // Send the final version of the previous local map
    vector <float> colors_v;
    float colors_a[3];
    colors_a[0] = pc_vis_->colors[3*cloud_counter];
    colors_a[1] = pc_vis_->colors[3*cloud_counter+1];
    colors_a[2] = pc_vis_->colors[3*cloud_counter+2];
    colors_v.assign(colors_a,colors_a+4*sizeof(float));
    stringstream ss;
    ss << "Prior Cloud - Local Map " << cloud_counter;
    int map_coll_id = cloud_counter + 10000 + 1; // so that first is 1001 and null is 1000
    ptcld_cfg pcfg = ptcld_cfg(map_coll_id,    ss.str()     ,1,1, 1000,1,colors_v);

    pc_vis_->pose_to_lcm_from_list(1000, null_poseT);
    pc_vis_->ptcld_to_lcm(pcfg, *accum_cloud, null_poseT.utime, null_poseT.utime );
        
    
    
    newmap_requested=0;
  }
}

int main(int argc, char ** argv){
  lcm_t * lcm;
  lcm = lcm_create(NULL);
  local_map app(lcm);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}