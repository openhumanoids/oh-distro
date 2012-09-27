// Listen to map_store_POINTS files:
// - store them in a local object
// - maintain a list and broadcast the list
// - support queries, which sent a specific cloud back
// - listen for 



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


#include "map_store.hpp"

using namespace std;
using namespace pcl;





/////////////////////////////////////

map_store::map_store(lcm_t* publish_lcm):
          publish_lcm_(publish_lcm) {




  pc_vis_ = new pointcloud_vis(publish_lcm_);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - Dump",5,0) );
  float colors_b[] ={0.0,0.0,1.0};
  vector <float> colors_v;
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - Dump"     ,1,0, 1000,1,colors_v));




  // LCM:
  lcm_t* subscribe_lcm_ = publish_lcm_;
  drc_pointcloud2_t_subscribe(subscribe_lcm_, "LOCAL_MAP_POINTS",
      pointcloud_handler_aux, this);

  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "SEG_REQUEST",
      seg_request_handler_aux, this);

  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "SEG_UPDATE",
      seg_update_handler_aux, this);

  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "DUMP_MAPS",
      dump_maps_handler_aux, this);

  drc_localize_reinitialize_cmd_t_subscribe(subscribe_lcm_, "CURRENT_MAP_ID",
      current_map_handler_aux, this);
}


void map_store::seg_request_handler(const drc_localize_reinitialize_cmd_t *msg){
  cout << "segmentation map requested\n";

  // ... transmit the map to the segmentation gui
}


void map_store::seg_update_handler(const drc_localize_reinitialize_cmd_t *msg){
  cout << "received segmentation\n";

  // ... log the segmentation
}

void map_store::dump_maps(DumpCode code, double x_offset){

  if (code == DUMP_SCREEN){
   cout << "dump screen set\n";
  }

  cout << "dump maps blah\n";
  for (size_t i=0;i<maps.size() ; i++){
     LocalMap m = maps[i];
    cout <<m.map_id  << " | " << m.utime << "|" << m.cloud->size() << " cloud size\n";

    Eigen::Isometry3d offset_pose = m.base_pose;
    offset_pose.translation() << m.base_pose.translation().x() + x_offset*i,
               m.base_pose.translation().y()+ 40,
               m.base_pose.translation().z();

    Isometry3dTime offset_poseT = Isometry3dTime(m.utime, offset_pose);

    pc_vis_->pose_to_lcm_from_list(1000, offset_poseT);
    pc_vis_->ptcld_to_lcm_from_list(1001, *(m.cloud), offset_poseT.utime, offset_poseT.utime);



  }
}

void map_store::dump_maps_handler(const drc_localize_reinitialize_cmd_t *msg){
  cout << "dump maps requested\n";

  // ... transmit the map to the segmentation gui

  dump_maps(DUMP_SCREEN,40.0);
}

void map_store::current_map_handler(const drc_localize_reinitialize_cmd_t *msg){
  cout << "map\n";

  // contains which map to visualise
  // ... transmit the specified map to the segmentation gui (continously @ 1Hz in a different thread)
}

void map_store::pointcloud_handler(const drc_pointcloud2_t *msg){

  cout << "got ptd handler\n";
  // Create new local map object, store cloud

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pc_lcm_->unpack_pointcloud2( (const ptools_pointcloud2_t*)   msg, cloud);


  LocalMap map;
  //Eigen::Isometry3d base_pose;
  map.map_id =maps.size(); // use a counter instead
  map.utime =maps.size();// msg->utime; // use time later..

  map.base_pose.setIdentity();
  map.cloud =cloud;



  maps.push_back(map);
  cout << maps.size() << "maps now stored\n";


}

int
main(int argc, char ** argv)
{
  lcm_t * lcm;
  lcm = lcm_create(NULL);
  map_store app(lcm);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}

