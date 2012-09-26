#ifndef local_map_velodyne_HPP_
#define local_map_velodyne_HPP_

#include <lcm/lcm.h>

#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>
#include <lcmtypes/drc_lcmtypes.h>

///////////////////////////////////////////////////////////////
class local_map{
  public:
    local_map(lcm_t* publish_lcm,lcm_t* subscribe_lcm);


    int initialize(int argc, char **argv);
    
    ~local_map(){
    }
    
  private:
    lcm_t* publish_lcm_;
    lcm_t* subscribe_lcm_;
    BotParam *param;
    BotFrames *frames;


    velodyne_calib_t *calib;
    velodyne_laser_return_collector_t *collector;

    bot_core_pose_t *bot_pose_last;
    BotPtrCircular   *velodyne_data_circ;

    int64_t         last_velodyne_data_utime;
    int64_t           last_pose_utime;




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

    int process_velodyne (const senlcm_velodyne_t *v);

    // Convert Circular Buffer into PCL:
    void unpack_velodyne(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

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

    static void velodyne_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const senlcm_velodyne_list_t* msg,
                                void* user_data) {
      ((local_map *) user_data)->velodyne_handler(msg);
    }
    void velodyne_handler(const senlcm_velodyne_list_t *msg);


    static void pose_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_pose_t* msg,
                                void* user_data) {
      ((local_map *) user_data)->pose_handler(msg);
    }
    void pose_handler(const bot_core_pose_t *msg);

};    

#endif
