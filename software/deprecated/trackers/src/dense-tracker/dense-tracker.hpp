#ifndef ICP_TRACKER_
#define ICP_TRACKER_

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

class ICPTracker
{
  public:
    typedef boost::shared_ptr<ICPTracker> Ptr;
    typedef boost::shared_ptr<const ICPTracker> ConstPtr;
        
    ICPTracker (boost::shared_ptr<lcm::LCM> &lcm_, int verbose_lcm_);

    ~ICPTracker() {
    }


    // Set the bounding box of the object to be tracked:
    void setBoundingBox( Eigen::Vector3f & boundbox_lower_left_in, Eigen::Vector3f & boundbox_upper_right_in );
    
    void drawBoundingBox(Eigen::Isometry3f pose);
    
    void boundingBoxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Eigen::Isometry3f pose);

    void removePoseOffset(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &previous_cloud, 
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_cloud, 
			Eigen::Isometry3f previous_pose );

    bool doICP( pcl::PointCloud<pcl::PointXYZRGB>::Ptr &previous_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_cloud,
			Eigen::Matrix4f & tf_previous_to_new);

    void doICPTracker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &previous_cloud, 
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_cloud, Eigen::Isometry3d previous_pose);
    
    Eigen::Isometry3d getUpdatedPose(){
      return new_pose_;
    };
  private:
    int verbose_lcm_; // 0 say nothing, 1 say important, 2 say lots

    boost::shared_ptr<lcm::LCM> lcm_;
    pointcloud_vis* pc_vis_;

    Isometry3dTime null_poseT_;    

    Eigen::Isometry3d new_pose_; 
    
    Eigen::Vector3f boundbox_lower_left_;
    Eigen::Vector3f boundbox_upper_right_;

};




#endif
