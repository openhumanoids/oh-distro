#include <zlib.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <boost/shared_ptr.hpp>
#include "lidar-odometry.hpp"

#include <ConciseArgs>

#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

using namespace std;

struct CommandLineConfig
{
  bool verbose;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);
    
    ~App(){
    }

  private:
    const CommandLineConfig cl_cfg_;    
    boost::shared_ptr<lcm::LCM> lcm_;

    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;

    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);
    LidarOdom* lidarOdom_;
    Eigen::Isometry3d world_to_body_;
};    

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_) : 
       lcm_(lcm_), cl_cfg_(cl_cfg_){
  lcm_->subscribe("SCAN",&App::lidarHandler,this);

  // Set up frames and config:
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  botframes_cpp_ = new bot::frames(botframes_);

  lidarOdom_ = new LidarOdom(lcm_);
}

static inline bot_core::pose_t getPoseAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();
  Eigen::Quaterniond r_x(pose.rotation());
  pose_msg.orientation[0] =  r_x.w();
  pose_msg.orientation[1] =  r_x.x();
  pose_msg.orientation[2] =  r_x.y();
  pose_msg.orientation[3] =  r_x.z();
  return pose_msg;
}


void App::lidarHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::planar_lidar_t* msg){
  const float* ranges = &msg->ranges[0];
  lidarOdom_->doOdometry(ranges, msg->nranges, msg->rad0, msg->radstep, msg->utime);

  bot_core::pose_t pose_msg = getPoseAsBotPose( lidarOdom_->getCurrentPose() , msg->utime);
  // lcm_->publish("POSE_SCAN_ALT", &pose_msg );

  // Determine the body position in world frame:
  Eigen::Isometry3d lidar_to_body = botframes_cpp_->get_trans_with_utime( botframes_ ,  "body", "SCAN"  , msg->utime);
  world_to_body_ = lidarOdom_->getCurrentPose() * lidar_to_body;
  bot_core::pose_t pose_msg_body = getPoseAsBotPose( world_to_body_ , msg->utime);
  lcm_->publish("POSE_BODY", &pose_msg_body );
}

int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.verbose = false;

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.verbose, "v", "verbose", "Verbose printf");
  parser.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  App app= App(lcm, cl_cfg);
  while(0 == lcm->handle());
}
