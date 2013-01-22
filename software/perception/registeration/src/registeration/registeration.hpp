#ifndef registeration_HPP_
#define registeration_HPP_

#include <lcm/lcm.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>
#include <lcmtypes/drc_lcmtypes.h>


/**
 * Represent a single image feature.
 */
struct ImageFeature
{
  int track_id;
  Eigen::Vector2d uv; ///< unrectified, distorted, orig. coords
  Eigen::Vector2d base_uv; ///< unrectified, distorted, base level
  Eigen::Vector3d uvd; ///< rectified, undistorted, base level
  Eigen::Vector3d xyz;
  Eigen::Vector4d xyzw;
  uint8_t color[3];

  // @todo what more is needed?
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


///////////////////////////////////////////////////////////////
class registeration{
  public:
    registeration(lcm_t* publish_lcm);

    void go();

    
    ~registeration(){
    }
    
  private:
    lcm_t* publish_lcm_;

    pointcloud_lcm* pc_lcm_;
    pointcloud_vis* pc_vis_;

    Isometry3dTime current_poseT;
    bool current_pose_init; // have we started
    Isometry3dTime null_poseT;
    Isometry3dTime local_poseT; // LIDAR pose where we started the most recent local map

    // Fixed transform [initally hardcoded]:
    Eigen::Isometry3d camera_to_lidar;

    // Current submap clouds
    pcl::PointCloud<PointXYZRGB>::Ptr cloud;
    int cloud_counter;


    // Pair 0:
    cv::Mat descriptors0, descriptors1;
    std::vector<bool> valid0,valid1; // validity of descriptors? not sure for what

    std::vector<ImageFeature> features0,features1;

    /// Minimum number of inliers needed.
    int min_inliers_;

    static void newmap_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_localize_reinitialize_cmd_t* msg,
                                void* user_data){
      ((registeration *) user_data)->newmap_handler(msg);
    }
    void newmap_handler(const drc_localize_reinitialize_cmd_t *msg);

    static void pointcloud_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_pointcloud2_t* msg,
                                void* user_data) {
      ((registeration *) user_data)->pointcloud_handler(msg);
    }
    void pointcloud_handler(const drc_pointcloud2_t *msg);

    static void lidar_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_planar_lidar_t* msg,
                                void* user_data) {
      ((registeration *) user_data)->lidar_handler(msg);
    }
    void lidar_handler(const bot_core_planar_lidar_t *msg);


    static void pose_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_pose_t* msg,
                                void* user_data) {
      ((registeration *) user_data)->pose_handler(msg);
    }
    void pose_handler(const bot_core_pose_t *msg);

};    

#endif
