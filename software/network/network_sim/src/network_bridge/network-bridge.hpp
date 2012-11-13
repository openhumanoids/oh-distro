#ifndef network_bridge_HPP_
#define network_bridge_HPP_

#include <lcm/lcm.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/drc_lcmtypes.h>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution



struct Resend{
  Resend(std::string channel, double max_freq, int64_t last_utime):
    channel(channel), max_freq(max_freq), last_utime(last_utime) {}
  std::string channel; // .. LCM channel
  double max_freq; // max freq of transmission
  int64_t last_utime; // last utime of transmission
};


///////////////////////////////////////////////////////////////
class network_bridge{
  public:
    network_bridge(lcm_t* publish_lcm,lcm_t* subscribe_lcm);
    
    ~network_bridge(){
    }

  private:

    lcm_t* publish_lcm_;
    lcm_t* subscribe_lcm_;
    
    std::vector<Resend> resendlist_;
    int determine_resend_from_list(std::string channel, int64_t msg_utime);
    

    static void robot_state_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_robot_state_t* msg,
                                void* user_data) {
      ((network_bridge *) user_data)->robot_state_handler(msg,channel);
    }
    void robot_state_handler(const drc_robot_state_t *msg,const char* channel);

    static void pose_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_pose_t* msg,
                                void* user_data) {
      ((network_bridge *) user_data)->pose_handler(msg,channel);
    }
    void pose_handler(const bot_core_pose_t *msg,const char* channel);
    
    static void image_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_image_t* msg,
                                void* user_data) {
      ((network_bridge *) user_data)->image_handler(msg,channel);
    }
    void image_handler(const bot_core_image_t *msg,const char* channel);  

    static void heightmap_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const drc_heightmap_t* msg,
                                void* user_data) {
      ((network_bridge *) user_data)->heightmap_handler(msg,channel);
    }
    void heightmap_handler(const drc_heightmap_t *msg,const char* channel);      
};    

#endif
