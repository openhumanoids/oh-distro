#ifndef simple_segment_HPP_
#define simple_segment_HPP_

#include <lcm/lcm.h>

#include <pointcloud_utils/pointcloud_lcm.hpp>
#include <pointcloud_utils/pointcloud_vis.hpp>

///////////////////////////////////////////////////////////////
class simple_segment{
  public:
    simple_segment(lcm_t* publish_lcm);
    
    ~simple_segment(){
    }

    inline void set_input_cloud (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input){
      incloud   = input;
    }


    void do_segment ();

  private:

    
    void find_and_remove_plane(pcl::PointCloud<PointT>::Ptr cloud_filtered,
			pcl::PointCloud<PointT>::Ptr cloud_filtered2,
			pcl::PointCloud<PointT>::Ptr cloud_plane,
			double threshold,
			pcl::ModelCoefficients::Ptr coeff);

    void find_cylinder(pcl::PointCloud<PointT>::Ptr cloud_filtered,
			   pcl::PointCloud<PointT>::Ptr cloud_filtered2, double threshold);

    void box_filter (pcl::PointCloud<PointT>::Ptr input);

    lcm_t* publish_lcm_;

    pointcloud_lcm* pc_lcm_;
    pointcloud_vis* pc_vis_;

    Isometry3dTime null_poseT;

    // Fixed transform [initally hardcoded]:
    Eigen::Isometry3d camera_to_lidar;

    // Current submap clouds
    pcl::PointCloud<PointXYZRGB>::Ptr incloud;

};    

#endif
