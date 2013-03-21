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

    // Determine the current estimate of the plane
    bool getPlane(Eigen::Isometry3d &plane_pose , int64_t current_utime_);
    
    // set the plane to be tracked:
    void setPlane(std::vector<float> plane_coeffs, Eigen::Vector4f plane_centroid ){
      pcl::ModelCoefficients::Ptr new_plane_coeffs(new pcl::ModelCoefficients ());
      new_plane_coeffs->values = plane_coeffs;
      plane_pose_ = determinePlanePose(new_plane_coeffs, plane_centroid);
      plane_coeffs_ = new_plane_coeffs;
      plane_pose_init_ = true;
      cout << "[PLANE] Have initialized the plane tracker\n";
    };

    // Given a plane coeff and a centroid, determine a pose on the plane at the centroid:
    Eigen::Isometry3d determinePlanePose(pcl::ModelCoefficients::Ptr plane_coeffs,
          Eigen::Vector4f centroid);
    
  private:
    int verbose_lcm_; // 0 say nothing, 1 say important, 2 say lots

    // Plane Detection:
    bool getSweep();
    void findPlane();
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
