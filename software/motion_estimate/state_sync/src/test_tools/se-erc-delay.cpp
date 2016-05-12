#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <path_util/path_util.h>
#include <lcmtypes/bot_core.hpp>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/robot_state_t.hpp>

#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <ConciseArgs>

////////////////////////////////////////
struct CommandLineConfig
{
    int mode;
    int delay_of_pose;
    std::string input_channel;
    std::string output_channel;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_);
    
    ~App(){
    }    
    
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig& ca_cfg_;
    
    void pbdih(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::robot_state_t* msg);   
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   
    
    int counter_; // used for terminal feedback
    int verbose_;
};


App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_):
    lcm_(lcm_), ca_cfg_(ca_cfg_){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  botframes_cpp_ = new bot::frames(botframes_);
  
  counter_ =0;  
  verbose_=3; // 1 important, 2 useful 3, lots
  
  std::cout << ca_cfg_.input_channel << " to " <<  ca_cfg_.output_channel << "\n";
  lcm_->subscribe(ca_cfg_.input_channel ,&App::pbdih,this);  
}

 
  
void App::pbdih(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::robot_state_t* msg){
  std::cout << "\n" << msg->utime << "\n";

  int64_t time_in_past = msg->utime - ca_cfg_.delay_of_pose*1E6;
  Eigen::Isometry3d local_to_body;

  int status = botframes_cpp_->get_trans_with_utime( botframes_ ,  "body", "local", time_in_past, local_to_body);
  std::cout << status << "\n";

  bot_core::robot_state_t m_out = *msg;

  m_out.pose.translation.x  = local_to_body.translation().x();
  m_out.pose.translation.y  = local_to_body.translation().y();
  m_out.pose.translation.z  = local_to_body.translation().z();

  Eigen::Quaterniond quat = Eigen::Quaterniond(local_to_body.rotation());
  m_out.pose.rotation.w = quat.w();
  m_out.pose.rotation.x = quat.x();
  m_out.pose.rotation.y = quat.y();
  m_out.pose.rotation.z = quat.z();
  lcm_->publish(ca_cfg_.output_channel, &m_out);
}


int main(int argc, char ** argv) {
  CommandLineConfig ca_cfg;
  ca_cfg.mode = 0;
  ca_cfg.delay_of_pose = 30; // delay visualised pose by XX seconds
  ca_cfg.input_channel = "EST_ROBOT_STATE_ALT";
  ca_cfg.output_channel = "EST_ROBOT_STATE";

  ConciseArgs opt(argc, (char**)argv);
  opt.add(ca_cfg.mode , "m", "type","Mode: 0 spit params. 1 push");
  opt.add(ca_cfg.delay_of_pose , "d", "delay","number of seconds to delay the ERS");
  opt.parse();  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App accu(lcm, ca_cfg);
  
  while(0 == lcm->handle());  

  return 0;
}
