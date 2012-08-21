#ifndef LOCAL_MAP_HPP_
#define LOCAL_MAP_HPP_

#include <lcm/lcm.h>


#include <pointcloud_utils/pointcloud_lcm.hpp>

#include <pointcloud_utils/pointcloud_vis.hpp>

///////////////////////////////////////////////////////////////
class local_map{
  public:
    local_map(lcm_t* publish_lcm);
    
    ~local_map(){
    }
    
  private:
    lcm_t* publish_lcm_;

    pointcloud_vis* pc_vis_;

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


};    

#endif
