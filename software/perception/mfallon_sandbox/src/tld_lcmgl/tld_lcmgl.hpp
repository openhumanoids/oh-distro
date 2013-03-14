#ifndef tld_lcmgl_hpp_
#define tld_lcmgl_hpp_

#include <GL/gl.h>
#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <bot_param/param_util.h>
///////////////////////////////////////////////////////////////

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pointcloud_tools/pointcloud_math.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/tld_tracker.hpp>


class tld_lcmgl{
  public:
    tld_lcmgl(boost::shared_ptr<lcm::LCM> &lcm_);


    int do_some_lcmgl();
    
    ~tld_lcmgl(){
    }

      
  private:
    BotParam *param;
    BotFrames *frames;
    bot::frames* frames_cpp;
    
    
    boost::shared_ptr<lcm::LCM> lcm_;
    bot_lcmgl_t* lcmgl_;

    void on_seek_goal(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::seek_goal_timed_t* msg);
    void on_pointing_vector(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  perception::pointing_vector_t* msg);
    
    int64_t seek_timeout_;
    int64_t seek_utime_;
    int8_t seek_type_;
};    

#endif
