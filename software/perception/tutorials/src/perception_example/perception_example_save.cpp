// LCM example program for interacting  with PCL data
// In this case it pushes the data to an (ascii) file
// as well as putting some details of the cloud to screen
// mfallon aug 2012
#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <lcmtypes/drc_lcmtypes.h>

using namespace std;
using namespace pcl;

lcm_t * lcm;



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


  cout << "     utime: " << msg->utime << "\n";
  cout << "    height: " << msg->height << "\n";
  cout << "     width: " << msg->width  << "\n";
  cout << "num_points: " << num_points << "\n";
  cout << "    nbytes: " << msg->data_nbytes << "\n";

  pcl::PCDWriter writer;
  writer.write ("example_output.pcd", *cloud, false);
  std::cerr << "Saved " << cloud->points.size () << " data points to file." << std::endl;
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

