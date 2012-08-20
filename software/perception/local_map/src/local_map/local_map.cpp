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

using namespace std;
using namespace pcl;

lcm_t * lcm;

// Basic publisher for data to collections viewer:
bool pcdXYZRGB_to_lcm(lcm_t *lcm, pcl::PointCloud<pcl::PointXYZRGB> &cloud,
    int64_t msg_time,int pose_collection_id, int64_t pose_id,
    bool reset){

  int npoints =cloud.points.size();
  //int pose_id = 0; // use timestamp

  int64_t point_lists_id = pose_id; // use timestamp
  int point_type = 1; // point
  string collection_name = "Example Cloud";
  int cloud_collection = 401;

  vs_point3d_list_collection_t plist_coll;
  plist_coll.id = cloud_collection;
  plist_coll.name =(char*)   collection_name.c_str();
  plist_coll.type =point_type; // collection of points
  plist_coll.reset = reset;
  plist_coll.nlists = 1; // number of seperate sets of points
  vs_point3d_list_t plist[plist_coll.nlists];

  // loop here for many lists
  vs_point3d_list_t* this_plist = &(plist[0]);
  // 3.0: header
  this_plist->id =point_lists_id; //bot_timestamp_now();
  this_plist->collection = pose_collection_id;
  this_plist->element_id = pose_id;
  // 3.1: points/entries (rename)
  vs_point3d_t* points = new vs_point3d_t[npoints];
  this_plist->npoints = npoints;
  // 3.2: colors:
  vs_color_t* colors = new vs_color_t[npoints];
  this_plist->ncolors = npoints;
  // 3.3: normals and ids:
  this_plist->nnormals = 0;
  this_plist->normals = NULL;
  this_plist->npointids = 0;
  this_plist->pointids= NULL;

  for(int j=0; j<npoints; j++) {
      colors[j].r = cloud.points[j].r/255.0; // points_collection values range 0-1
      colors[j].g = cloud.points[j].g/255.0;
      colors[j].b = cloud.points[j].b/255.0;
      points[j].x = cloud.points[j].z;
      points[j].y = - cloud.points[j].x;
      points[j].z = - cloud.points[j].y;

  }
  this_plist->colors = colors;
  this_plist->points = points;
  plist_coll.point_lists = plist;
  vs_point3d_list_collection_t_publish(lcm,"POINTS_COLLECTION",&plist_coll);

  delete colors;
  delete points;
}



static void on_pointcloud(const lcm_recv_buf_t *rbuf, const char * channel, const drc_pointcloud2_t * msg, void * user){
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

  // 3. Transmit data to viewer at null pose:
  // 3a. Transmit Pose:
  int pose_id=1;
  bool reset = true;

  int null_obj_collection= 400;
  // Send a pose to hang the model on:
  vs_obj_collection_t objs;
  objs.id = null_obj_collection;
  objs.name = (char*)  "Zero Pose"; // "Trajectory";
  objs.type = 1; // a pose
  objs.reset = false; // true will delete them from the viewer
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
  pcdXYZRGB_to_lcm(lcm , *cloud, msg->utime,null_obj_collection,pose_id,reset);
}

int
main(int argc, char ** argv)
{
  lcm = lcm_create(NULL);
  if(!lcm)
    return 1;

  drc_pointcloud2_t_subscribe(lcm, "WIDE_STEREO_POINTS", &on_pointcloud, NULL);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}

