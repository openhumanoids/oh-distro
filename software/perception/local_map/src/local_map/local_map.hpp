#ifndef LOCAL_MAP_HPP_
#define LOCAL_MAP_HPP_

#include <lcm/lcm.h>

#include <pointcloud_utils/pointcloud_lcm.hpp>
#include <pointcloud_utils/pointcloud_vis.hpp>
#include <lcmtypes/drc_lcmtypes.h>

///////////////////////////////////////////////////////////////
class local_map{
  public:
    local_map(lcm_t* publish_lcm);
    
    ~local_map(){
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

    static void newmap_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_localize_reinitialize_cmd_t* msg,
                                void* user_data){
      ((local_map *) user_data)->newmap_handler(msg);
    }
    void newmap_handler(const drc_localize_reinitialize_cmd_t *msg);

    static void pointcloud_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_pointcloud2_t* msg,
                                void* user_data) {
      ((local_map *) user_data)->pointcloud_handler(msg);
    }
    void pointcloud_handler(const drc_pointcloud2_t *msg);

    static void lidar_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_planar_lidar_t* msg,
                                void* user_data) {
      ((local_map *) user_data)->lidar_handler(msg);
    }
    void lidar_handler(const bot_core_planar_lidar_t *msg);


    static void pose_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_pose_t* msg,
                                void* user_data) {
      ((local_map *) user_data)->pose_handler(msg);
    }
    void pose_handler(const bot_core_pose_t *msg);

};    

#endif
