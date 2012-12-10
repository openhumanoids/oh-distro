#ifndef heightmap_lcmgl_hpp_
#define heightmap_lcmgl_hpp_

#include <GL/gl.h>
#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <Eigen/Geometry>

#include <lcmtypes/drc_heightmap_t.h>
#include <lcmtypes/drc_map_params_t.h>
///////////////////////////////////////////////////////////////
class heightmap_lcmgl{
  public:
    heightmap_lcmgl(lcm_t* publish_lcm,lcm_t* subscribe_lcm);


    int initialize(int argc, char **argv);
    
    ~heightmap_lcmgl(){
    }
    
  private:
    lcm_t* publish_lcm_;
    lcm_t* subscribe_lcm_;
    bot_lcmgl_t* lcmgl_;

    BotParam *param;
    BotFrames *frames;


    bot_core_pose_t *bot_pose_last;

    int64_t         last_velodyne_data_utime;
    int64_t           last_pose_utime;

    double hack_offset_x_;
    double hack_offset_y_;
    double hack_offset_z_;
/*    
    Isometry3dTime current_poseT;
    bool current_pose_init; // have we started
    Isometry3dTime null_poseT;
    Isometry3dTime local_poseT; // LIDAR pose where we started the most recent local map
    
    // Fixed transform [initally hardcoded]:
    Eigen::Isometry3d camera_to_lidar;
    */

    // Current submap clouds
    //pcl::PointCloud<PointXYZRGB>::Ptr cloud;
    int cloud_counter;

    uint8_t* rgb_buf_;
    int rgb_buf_size_;

    static void heightmap_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_heightmap_t* msg,
                                void* user_data){
      ((heightmap_lcmgl *) user_data)->heightmap_handler(msg);
    }
    void heightmap_handler(const drc_heightmap_t *msg);


    static void mapcreate_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_map_params_t* msg,
                                void* user_data){
      ((heightmap_lcmgl *) user_data)->mapcreate_handler(msg);
    }
    void mapcreate_handler(const drc_map_params_t *msg);
    
    static void image_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_image_t* msg,
                                void* user_data){
      ((heightmap_lcmgl *) user_data)->image_handler(msg);
    }
    void image_handler(const bot_core_image_t *msg);

    static void pose_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_pose_t* msg,
                                void* user_data) {
      ((heightmap_lcmgl *) user_data)->pose_handler(msg);
    }
    void pose_handler(const bot_core_pose_t *msg);

};    

#endif
