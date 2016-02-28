#ifndef PCL_CONVERSIONS_LCM_H_
#define PCL_CONVERSIONS_LCM_H_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;

namespace pcl
{

  template <typename PointT> void
  fromLCMPointCloud2 (const bot_core::pointcloud2_t& msg, pcl::PointCloud<PointT>& cloud)
  {

    pcl::PCLPointCloud2 msg_blob;
 
    msg_blob.height = msg.height;
    msg_blob.width = msg.width;

    for (size_t i=0; i<msg.fields.size();i++){
      pcl::PCLPointField field;
      field.name = msg.fields[i].name;
      field.offset = msg.fields[i].offset;
      field.datatype = msg.fields[i].datatype;
      field.count = msg.fields[i].count;
      msg_blob.fields.push_back(field);
    }

    msg_blob.is_bigendian = msg.is_bigendian;
    msg_blob.point_step = msg.point_step;
    msg_blob.row_step = msg.row_step;
    msg_blob.data = msg.data;
    msg_blob.is_dense = msg.is_dense;

    pcl::fromPCLPointCloud2 (msg_blob, cloud);

  }
}

#endif  //#ifndef PCL_CONVERSIONS_H_
