#ifndef lcmgl_example_hpp_
#define lcmgl_example_hpp_

#include <GL/gl.h>
#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>

///////////////////////////////////////////////////////////////
class lcmgl_example{
  public:
    lcmgl_example(lcm_t* publish_lcm,lcm_t* subscribe_lcm);


    int do_some_lcmgl(int argc, char **argv);
    
    ~lcmgl_example(){
    }
    
  private:
    lcm_t* publish_lcm_;
    lcm_t* subscribe_lcm_;
    bot_lcmgl_t* lcmgl_;

};    

#endif
