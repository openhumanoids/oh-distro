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
      publish_lcm_(publish_lcm){


  pc_vis_ = new pointcloud_vis(publish_lcm_);  

  lcm_t* subscribe_lcm_ = publish_lcm_;
  drc_pointcloud2_t_subscribe(subscribe_lcm_, "WIDE_STEREO_POINTS",
                             local_map::pointcloud_handler_aux, this);


  bot_core_planar_lidar_t_subscribe(subscribe_lcm_, "BASE_SCAN",
                             local_map::lidar_handler_aux, this);

}

void local_map::lidar_handler(const bot_core_planar_lidar_t *msg){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());


//  double spatialDecimationThresh = 0.2;
//  int beam_skip=3;
//  smPoint * points = (smPoint *) calloc(msg->nranges, sizeof(smPoint));
  double maxRange = 29.7;
  double validBeamAngles[] ={-2.1,2.1};

  convertLidar(msg->ranges, msg->nranges, msg->rad0,
        msg->radstep, cloud, 29.7,
        validBeamAngles[0], validBeamAngles[1]);




 // 3. Transmit data to viewer at null pose:
  // 3a. Transmit Pose:
  int pose_id=msg->utime;
  bool reset = true;

  int null_obj_collection= 600;
  // Send a pose to hang the model on:
  vs_obj_collection_t objs;
  objs.id = null_obj_collection;
  objs.name = (char*)  "Zero Pose LIDAR"; // "Trajectory";
  objs.type = 5; // a pose
  objs.reset = true; // true will delete them from the viewer
  objs.nobjs = 1;
  vs_obj_t poses[objs.nobjs];
  poses[0].id = (int64_t) pose_id;// which specific pose
  // directly matches the frames in gazebo: 
  // <origin rpy="1.570796325 0 0" xyz="0.275 0.0 0.252"/>
  poses[0].x = 0.275;
  poses[0].y = 0.0;
  poses[0].z = 0.252; //0.275 0.0 0.252
  poses[0].yaw =  0;
  poses[0].pitch = 0;
  poses[0].roll = M_PI/2;
  objs.objs = poses;
  vs_obj_collection_t_publish(lcm, "OBJ_COLLECTION", &objs);

  // 3b. Send Data:
  // Send the Model, hanging on the zero-zero pose
  Ptcoll_cfg ptcoll_cfg;
  ptcoll_cfg.reset=true;
  ptcoll_cfg.point_lists_id =msg->utime;
  ptcoll_cfg.collection = null_obj_collection;
  ptcoll_cfg.element_id =(int64_t) pose_id;// which specific pose does this hang on .. all the same
  ptcoll_cfg.name.assign("Model LIDAR");  
  ptcoll_cfg.id = 601;
  ptcoll_cfg.npoints =  cloud->points.size();
  ptcoll_cfg.type =1;
  float colorm_temp0[] ={-10.0,-1.0,-1.0,-1.0};
  ptcoll_cfg.rgba.assign(colorm_temp0,colorm_temp0+4*sizeof(float));

  pc_vis_->pcdXYZRGB_to_lcm(ptcoll_cfg, *cloud);



}

void local_map::pointcloud_handler(const drc_pointcloud2_t *msg){

  // 1. Copy fields - this duplicates /pcl/ros/conversions.h for "fromROSmsg"
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud->width   = msg->width;
  cloud->height   = msg->height;
  uint32_t num_points = msg->width * msg->height;
  cloud->points.resize (num_points);
  cloud->is_dense = false;//msg->is_dense;
  uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&cloud->points[0]);
  uint32_t cloud_row_step = static_cast<uint32_t> (sizeof (pcl::PointXYZRGB) * cloud->width);
  const uint8_t* msg_data = &msg->data[0];
  memcpy (cloud_data, msg_data, msg->data_nbytes );

  // 2. HACK/Workaround
  // for some reason in pcl1.5/6, this callback results
  // in RGB data whose offset is not correctly understood
  // Instead of an offset of 12bytes, its offset is set to be 16
  // this fix corrects for the issue:
  sensor_msgs::PointCloud2 msg_cld;
  pcl::toROSMsg(*cloud, msg_cld);
  msg_cld.fields[3].offset = 12;
  pcl::fromROSMsg (msg_cld, *cloud);

  std::cerr << "Received Cloud with " << cloud->points.size () << " data points." << std::endl;

  // Transform cloud to that its in robotic frame:
  double x_temp;
  for(int j=0; j<cloud->points.size(); j++) {
    x_temp = cloud->points[j].x;
    cloud->points[j].x = cloud->points[j].z;
    cloud->points[j].z = - cloud->points[j].y;
    cloud->points[j].y = - x_temp;
  }

  // 3. Transmit data to viewer at null pose:
  // 3a. Transmit Pose:
  int pose_id=msg->utime;
  bool reset = true;

  int null_obj_collection= 400;
  // Send a pose to hang the model on:
  vs_obj_collection_t objs;
  objs.id = null_obj_collection;
  objs.name = (char*)  "Zero Pose"; // "Trajectory";
  objs.type = 5; // a pose
  objs.reset = true; // true will delete them from the viewer
  objs.nobjs = 1;
  vs_obj_t poses[objs.nobjs];
  poses[0].id = (int64_t) pose_id;// which specific pose
  poses[0].x = 0;
  poses[0].y = 0;
  poses[0].z = 0;
  poses[0].yaw = 0;
  poses[0].pitch = 0;
  poses[0].roll = 0;
  objs.objs = poses;
  vs_obj_collection_t_publish(lcm, "OBJ_COLLECTION", &objs);

  // 3b. Send Data:
  // Send the Model, hanging on the zero-zero pose
  Ptcoll_cfg ptcoll_cfg;
  ptcoll_cfg.reset=true;
  ptcoll_cfg.point_lists_id =msg->utime;
  ptcoll_cfg.collection = null_obj_collection;
  ptcoll_cfg.element_id =(int64_t) pose_id;// which specific pose does this hang on .. all the same
  ptcoll_cfg.name.assign("Model");  
  ptcoll_cfg.id = 401;
  ptcoll_cfg.npoints =  cloud->points.size();
  ptcoll_cfg.type =1;
  float colorm_temp0[] ={-10.0,-1.0,-1.0,-1.0};
  ptcoll_cfg.rgba.assign(colorm_temp0,colorm_temp0+4*sizeof(float));

  pc_vis_->pcdXYZRGB_to_lcm(ptcoll_cfg, *cloud);
}

int
main(int argc, char ** argv)
{

  lcm_t* publish_lcm=lcm_create(NULL);

  lcm = lcm_create(NULL);
  local_map app(lcm);//= local_map(publish_lcm);


  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}

