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
	
    MajorPlane (boost::shared_ptr<lcm::LCM> &lcm_);

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
    
  private:

    // Plane Detection:
    bool getSweep();
    void findPlane();
    int64_t last_sweep_time_;
    // Point Cloud of most recent sweep:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;    
    // pose of a point on the plane with pitch and yaw but roll =0
    Eigen::Isometry3d plane_pose_ ;
    // has the above value been set?
    bool plane_pose_set_;

    int64_t  current_utime_;
};




#endif
