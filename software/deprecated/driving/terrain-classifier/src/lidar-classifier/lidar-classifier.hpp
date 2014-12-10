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
class lidar_classifier{
  public:
    lidar_classifier(lcm_t* lcm_, bool verbose_, int vis_history_, int estop_threshold_);
    
    ~lidar_classifier(){
    }
    
  private:
    lcm_t* lcm_;
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* frames_cpp_;
    
    pointcloud_lcm* pc_lcm_;
    pointcloud_vis* pc_vis_;
    
    // all important parameter:
    int estop_threshold_;

    bool verbose_;
    int vis_history_;
    int vis_counter_;

    static void lidar_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_planar_lidar_t* msg,
                                void* user_data) {
      ((lidar_classifier *) user_data)->lidar_handler(msg);
    }
    void lidar_handler(const bot_core_planar_lidar_t *msg);

};    

#endif
