#ifndef rgbd_vis_hpp_
#define rgbd_vis_hpp_

#include <lcm/lcm.h>

#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>

#include <lcmtypes/kinect.h>

///////////////////////////////////////////////////////////////
class rgbd_vis{
  public:
    rgbd_vis(lcm_t* publish_lcm,lcm_t* subscribe_lcm);


    int initialize(int argc, char **argv);
    
    ~rgbd_vis(){
    }
    
  private:
    lcm_t* publish_lcm_;
    lcm_t* subscribe_lcm_;
    BotParam *param;
    BotFrames *frames;


    bot_core_pose_t *bot_pose_last;

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
    //pcl::PointCloud<PointXYZRGB>::Ptr cloud;
    int cloud_counter;

    uint8_t* rgb_buf_;
    int rgb_buf_size_;


    static void kframe_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const kinect_frame_msg_t* msg,
                                void* user_data){
      ((rgbd_vis *) user_data)->kframe_handler(msg);
    }
    void kframe_handler(const kinect_frame_msg_t *msg);

    /*
static void pointcloud_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_pointcloud2_t* msg,
                                void* user_data) {
      ((rgbd_vis *) user_data)->pointcloud_handler(msg);
    }
    void pointcloud_handler(const drc_pointcloud2_t *msg);
*/


    static void pose_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_pose_t* msg,
                                void* user_data) {
      ((rgbd_vis *) user_data)->pose_handler(msg);
    }
    void pose_handler(const bot_core_pose_t *msg);

};    

#endif
