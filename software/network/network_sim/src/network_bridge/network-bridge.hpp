#ifndef network_bridge_HPP_
#define network_bridge_HPP_

#include <lcm/lcm.h>
#include <lcmtypes/bot_core.h>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/math/distributions/normal.hpp> // for normal_distribution


///////////////////////////////////////////////////////////////
class network_bridge{
  public:
    network_bridge(lcm_t* publish_lcm,lcm_t* subscribe_lcm);
    
    ~network_bridge(){
    }

    void do_segment ();

  private:

    lcm_t* publish_lcm_;
    lcm_t* subscribe_lcm_;

    static void pose_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_pose_t* msg,
                                void* user_data) {
      ((network_bridge *) user_data)->pose_handler(msg);
    }
    void pose_handler(const bot_core_pose_t *msg);

};    

#endif
