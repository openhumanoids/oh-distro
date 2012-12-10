#ifndef lcmgl_example_hpp_
#define lcmgl_example_hpp_

#include <GL/gl.h>
#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <lcmtypes/vicon_drc.h>

///////////////////////////////////////////////////////////////
class lcmgl_teleop{
  public:
    lcmgl_teleop(lcm_t* publish_lcm,lcm_t* subscribe_lcm);


    int do_some_lcmgl(int argc, char **argv);
    
    ~lcmgl_teleop(){
    }
    
  private:
    lcm_t* publish_lcm_;
    lcm_t* subscribe_lcm_;
    bot_lcmgl_t* lcmgl_;


    static void vicon_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const viconstructs_vicon_t* msg,
                                void* user_data){
      ((lcmgl_teleop *) user_data)->vicon_handler(msg);
    }
    void vicon_handler(const viconstructs_vicon_t *msg);

};    

#endif
