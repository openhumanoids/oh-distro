#ifndef MAJOR_PLANE_DETECT_
#define MAJOR_PLANE_DETECT_

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

/// MAPS:
#include <maps/SensorDataReceiver.hpp>
#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/Collector.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/PointCloudView.hpp>
#include <maps/Utils.hpp>

#include <drc_utils/Clock.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

class MajorPlane
{
  public:
    typedef boost::shared_ptr<MajorPlane> Ptr;
    typedef boost::shared_ptr<const MajorPlane> ConstPtr;
	
    MajorPlane (boost::shared_ptr<lcm::LCM> &lcm_, int verbose_lcm_);

    ~MajorPlane() {
      bot_lcmgl_destroy(mLcmGl);
    }

    //boost::shared_ptr<lcm::LCM> mLcm;
    boost::shared_ptr<lcm::LCM> lcm_;
    maps::BotWrapper::Ptr mBotWrapper;
    boost::shared_ptr<maps::Collector> mCollector;
    int mActiveMapId;
    bot_lcmgl_t* mLcmGl;

    pointcloud_vis* pc_vis_;

    // Get a sweep of lidar
    // True if we have a new sweep, different from before
    bool getSweep( Eigen::Vector3f bounds_center = Eigen::Vector3f(0,0,0),  
			Eigen::Vector3f bounds_size = Eigen::Vector3f(1e10, 1e10, 1e10) );
    
    // Return the cloud calculated from the sweep above
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSweepCloud(){
      return cloud_; 
    }
    
    // Determine the current estimate of the plane
    bool trackPlane(Eigen::Isometry3d &plane_pose , int64_t current_utime_);

    
    std::vector<float> getPlaneCoeffs(){
      return plane_coeffs_->values; 
    }
    
    // set the plane to be tracked:
    void setPlane(std::vector<float> plane_coeffs, Eigen::Vector4f plane_centroid ){
      pcl::ModelCoefficients::Ptr new_plane_coeffs(new pcl::ModelCoefficients ());
      new_plane_coeffs->values = plane_coeffs;
      plane_pose_ = determinePlanePose(new_plane_coeffs, plane_centroid);
      plane_coeffs_ = new_plane_coeffs;
      plane_pose_init_ = true;
      cout << "[PLANE] Have initialized the plane tracker\n";
    };

    
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSweepCloudWithoutPlane(float dist_threshold){
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
     
     for (size_t i=0; i < cloud_->points.size() ; i++ ){
        pcl::PointXYZRGB pt = cloud_->points[i]; 
        float top =plane_coeffs_->values[0]* pt.x + plane_coeffs_->values[1]* pt.y + 
                    plane_coeffs_->values[2]* pt.z + plane_coeffs_->values[3];
        float bottom = sqrt (plane_coeffs_->values[0]* plane_coeffs_->values[0] + plane_coeffs_->values[1]*plane_coeffs_->values[1] +
                              plane_coeffs_->values[2]* plane_coeffs_->values[2] );
        float dist = fabs(top/bottom);
        if (dist > dist_threshold){
          cloud_out->points.push_back(pt);
        }
     }
     return cloud_out;
   }

    
    
    void storeNewPlane( pcl::ModelCoefficients::Ptr new_plane_coeffs, Eigen::Vector4f new_plane_centroid,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_plane_hull );
    
    // Given a plane coeff and a centroid, determine a pose on the plane at the centroid:
    Eigen::Isometry3d determinePlanePose(pcl::ModelCoefficients::Ptr plane_coeffs,
          Eigen::Vector4f centroid);
    
  private:
    int verbose_lcm_; // 0 say nothing, 1 say important, 2 say lots

    // Determine if the old plane matches the largest plane in the current sweep:
    void matchToLargestPlane();
    
    // Determine if the old plane matches any of the planes in the current sweep:
    // more expensive than matchToLargestPlane
    void matchToAllPlanes();
    
    int64_t last_sweep_time_;
    // Point Cloud of most recent sweep:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;    
    // pose of a point on the plane with pitch and yaw but roll =0
    Eigen::Isometry3d plane_pose_ ;
    pcl::ModelCoefficients::Ptr plane_coeffs_;
    // has the above value been set?
    bool plane_pose_init_;

    int64_t  current_utime_;
};




#endif
