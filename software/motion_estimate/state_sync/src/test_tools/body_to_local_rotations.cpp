#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <path_util/path_util.h>
#include <lcmtypes/bot_core.hpp>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <lcmtypes/bot_core/pose_t.hpp>

#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <ConciseArgs>

////////////////////////////////////////
struct CommandLineConfig
{
    bool mode;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_);
    
    ~App(){
    }    
    
    void sendUpdate();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig& ca_cfg_;
    
    void pbdih(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::pose_t* msg);   
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   
    
    int counter_; // used for terminal feedback
    int verbose_;
};


App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& ca_cfg_):
    lcm_(lcm_), ca_cfg_(ca_cfg_){


  
  counter_ =0;  
  verbose_=3; // 1 important, 2 useful 3, lots
  
  lcm_->subscribe("POSE_BDI",&App::pbdih,this);  
}

void App::sendUpdate(){
  
  std::cout << "Updating\n";
  double c= M_PI/180;
  double rpy[3] = {c*0.,c*10.,c*10.};
  double q[4];
  bot_roll_pitch_yaw_to_quat (rpy, q);

  double vel[3] = {1.,0., 0.};

  std::cout << "befor: " << vel[0] << " " << vel[1] << " " << vel[2] << "\n";

  bot_quat_rotate(q, vel);
  
  std::cout << "after: " << vel[0] << " " << vel[1] << " " << vel[2] << "\n";

}


void App::pbdih(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  std::cout << "\n" << msg->utime << "\n";

  bot_core::pose_t m = *msg;

  double q[4];
  memcpy(q, m.orientation, 4*sizeof(double));

  //q[1] = -q[1];
  //q[2] = -q[2];
  //q[3] = -q[3];

  //double rpy[3];
  //double c= M_PI/180;
  //bot_quat_to_roll_pitch_yaw (q,rpy);
  //std::cout << "rpyd  : " << rpy[0]/c << " " << rpy[1]/c << " " << rpy[2]/c << "\n";


  BotTrans bt;
  memset(&bt, 0, sizeof(bt));
  memcpy(bt.rot_quat,m.orientation, 4*sizeof(double));
  // memcpy(bt.trans_vec,m.pos, 3*sizeof(double));

  bot_trans_invert(&bt);


  double vel[3];
  memcpy(vel, m.vel, 3*sizeof(double));
  std::cout << "lbefor: " << vel[0] << " " << vel[1] << " " << vel[2] << "\n";
  //bot_quat_rotate(q, vel);
  bot_trans_apply_vec(&bt, msg->vel, vel);
  std::cout << "lafter: " << vel[0] << " " << vel[1] << " " << vel[2] << "\n";
  memcpy(m.vel, vel, 3*sizeof(double));



  double rotr[3];
  memcpy(rotr, m.rotation_rate, 3*sizeof(double));

  std::cout << "rbefor: " << rotr[0] << " " << rotr[1] << " " << rotr[2] << "\n";
  // bot_quat_rotate(q, rotr);
  //bot_trans_apply_vec(&bt, msg->rotation_rate, rotr);
  std::cout << "rafter: " << rotr[0] << " " << rotr[1] << " " << rotr[2] << "\n";
  memcpy(m.rotation_rate, rotr, 3*sizeof(double));


  lcm_->publish("POSE_VICON", &m);  


}


int main(int argc, char ** argv) {
  CommandLineConfig ca_cfg;
  ca_cfg.mode = 0;
 
  ConciseArgs opt(argc, (char**)argv);
  opt.add(ca_cfg.mode , "m", "type","Mode: 0 spit params. 1 push");
  opt.parse();  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App accu(lcm, ca_cfg);
  
  if (ca_cfg.mode == 0){
    while(0 == lcm->handle());  
  }else{
    accu.sendUpdate();
  }

  return 0;
}
