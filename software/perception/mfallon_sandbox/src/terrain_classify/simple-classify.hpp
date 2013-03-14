#ifndef LOCAL_MAP_HPP_
#define LOCAL_MAP_HPP_

#include <lcm/lcm.h>

#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>
#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/bot_core.h>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>


///////////////////////////////////////////////////////////////
class simple_classify{
  public:
    simple_classify(lcm_t* publish_lcm);
    
    ~simple_classify(){
    }
    
  private:
    lcm_t* publish_lcm_;
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* frames_cpp;
    
    pointcloud_lcm* pc_lcm_;
    pointcloud_vis* pc_vis_;

    Isometry3dTime current_pose_headT;
    Isometry3dTime current_pose_bodyT;
    Isometry3dTime null_poseT;
    Isometry3dTime local_poseT; // LIDAR pose where we started the most recent local map
    Eigen::Isometry3d local_to_lidar;
    int counter_;

    static void lidar_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_planar_lidar_t* msg,
                                void* user_data) {
      ((simple_classify *) user_data)->lidar_handler(msg);
    }
    void lidar_handler(const bot_core_planar_lidar_t *msg);


    static void pose_handler_aux(const lcm_recv_buf_t* rbuf, const char* channel, const bot_core_pose_t* msg, void* user_data) {
      ((simple_classify *) user_data)->pose_handler(msg);
    }
    void pose_handler(const bot_core_pose_t *msg);

    static void pose_body_handler_aux(const lcm_recv_buf_t* rbuf, const char* channel, const bot_core_pose_t* msg, void* user_data) {
      ((simple_classify *) user_data)->pose_body_handler(msg);
    }
    void pose_body_handler(const bot_core_pose_t *msg);
};    

#endif