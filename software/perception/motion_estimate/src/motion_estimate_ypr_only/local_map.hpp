#ifndef LOCAL_MAP_HPP_
#define LOCAL_MAP_HPP_

#include <lcm/lcm.h>

#include <lcmtypes/bot_core.h>


///////////////////////////////////////////////////////////////
class local_map{
  public:
    local_map(lcm_t* publish_lcm);
    
    ~local_map(){
    }
    
  private:
    lcm_t* publish_lcm_;


    static void pose_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_pose_t* msg,
                                void* user_data) {
      ((local_map *) user_data)->pose_handler(msg);
    }
    void pose_handler(const bot_core_pose_t *msg);

};    

#endif
