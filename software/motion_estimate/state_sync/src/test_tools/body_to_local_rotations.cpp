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
    int mode;
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
  
  if (ca_cfg_.mode ==0){
    std::cout << "POSE_BDI local_linear_rate_to_body_linear_rate\n"; 
    lcm_->subscribe("POSE_BDI",&App::pbdih,this);  
  }else if(ca_cfg_.mode ==1){
    std::cout << "POSE_BODY body_linear_rate_to_local_linear_rate\n";
    lcm_->subscribe("POSE_BODY",&App::pbdih,this);  
  }
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



void local_linear_rate_to_body_linear_rate(const BotTrans *pose, const double lin_vel[3], double body_vel[3]){
  
  BotTrans bt;
  memset(&bt, 0, sizeof(bt)); /// didn't assign the position
  memcpy(bt.rot_quat,pose->rot_quat, 4*sizeof(double));
  
  bot_trans_invert(&bt);
  //std::cout << "lbefor: " << lin_vel[0] << " " << lin_vel[1] << " " << lin_vel[2] << "\n";
  //bot_quat_rotate(q, vel); // dont use
  
  bot_trans_apply_vec(&bt, lin_vel, body_vel);
  //  bot_quat_rotate_to(pose->rot_quat, lin_vel, body_vel);   // not right
  
  //std::cout << "lafter: " << body_vel[0] << " " << body_vel[1] << " " << body_vel[2] << "\n";
}
  
  
void body_linear_rate_to_local_linear_rate(const BotTrans *pose, const double body_vel[3], double lin_vel[3]){
  
  BotTrans bt;
  memset(&bt, 0, sizeof(bt)); /// didn't assign the position
  memcpy(bt.rot_quat,pose->rot_quat, 4*sizeof(double));
  
  //std::cout << "lbefor: " << body_vel[0] << " " << body_vel[1] << " " << body_vel[2] << "\n";
  bot_trans_apply_vec(&bt, body_vel, lin_vel);
  //std::cout << "lafter: " << lin_vel[0] << " " << lin_vel[1] << " " << lin_vel[2] << "\n";
}  
  
  
  
// Currently:
// BDI: linear velocity doesn't need to be converted, angular needs to go from body to linear
// MIT: linear velocity needs to go from body to linear, angular needs to go from body to linear


void App::pbdih(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  std::cout << "\n" << msg->utime << "\n";

  bot_core::pose_t m = *msg;
  
  BotTrans pose;
  memset(&pose, 0, sizeof(pose));
  memcpy(pose.rot_quat,msg->orientation, 4*sizeof(double));
  memcpy(pose.trans_vec,msg->pos, 3*sizeof(double));

//  bot_trans_invert(&bt);

  
  if (ca_cfg_.mode == 0){
    // Conversion from local_linear_rate_to_body_linear_rate
    
    // libbot version
    double lin_vel[3];
    double body_vel[3];
    memcpy(lin_vel,msg->vel, 3*sizeof(double));
    local_linear_rate_to_body_linear_rate(&pose, lin_vel, body_vel)  ;
    //std::cout << m.vel[0] << " " << m.vel[1] << " " << m.vel[2] << " body_vel (libbot)\n";
    //memcpy(m.vel, body_vel, 3*sizeof(double));
    
    Eigen::Matrix3d R = Eigen::Matrix3d( Eigen::Quaterniond( msg->orientation[0], msg->orientation[1], msg->orientation[2],msg->orientation[3] ));
    Eigen::Vector3d body_vel_E  = R.inverse()*Eigen::Vector3d ( msg->vel[0], msg->vel[1], msg->vel[2]);
    //std::cout << body_vel_E.transpose() << " body_vel (eigen)\n";
    memcpy(m.vel, body_vel_E.data(), 3*sizeof(double));
    
    // NB: angular rate is all ready in body frame
    
    // both are now in body frame
    
  }else if (ca_cfg_.mode == 1){
    // Conversion from body_linear_rate_to_local_linear_rate
    
    // libbot version
    double lin_vel[3];
    double body_vel[3];
    memcpy(body_vel,msg->vel, 3*sizeof(double));
    body_linear_rate_to_local_linear_rate(&pose, body_vel, lin_vel)  ;
    //std::cout << m.vel[0] << " " << m.vel[1] << " " << m.vel[2] << " lin_vel (libbot)\n";
    //memcpy(m.vel, lin_vel, 3*sizeof(double));
    
    
    Eigen::Matrix3d R = Eigen::Matrix3d( Eigen::Quaterniond( msg->orientation[0], msg->orientation[1], msg->orientation[2],msg->orientation[3] ));
    Eigen::Vector3d lin_vel_E  = R*Eigen::Vector3d ( msg->vel[0], msg->vel[1], msg->vel[2]);
    memcpy(m.vel, lin_vel_E.data(), 3*sizeof(double));
    //std::cout << body_vel_E.transpose() << " lin_vel (eigen)\n";
    
    Eigen::Vector3d rot_vel_E  = R*Eigen::Vector3d ( msg->rotation_rate[0], msg->rotation_rate[1], msg->rotation_rate[2]);
    memcpy(m.rotation_rate, rot_vel_E.data(), 3*sizeof(double));
    
    // Both are now in local frame
  }else if (ca_cfg_.mode == 2){
    
    // NB: linear rate is all realy in local frame

    Eigen::Matrix3d R = Eigen::Matrix3d( Eigen::Quaterniond( msg->orientation[0], msg->orientation[1], msg->orientation[2],msg->orientation[3] ));
    Eigen::Vector3d rot_vel_E  = R*Eigen::Vector3d ( msg->rotation_rate[0], msg->rotation_rate[1], msg->rotation_rate[2]);
    memcpy(m.rotation_rate, rot_vel_E.data(), 3*sizeof(double));

    // Both are now in local frame    
    
  }

  

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
  
  while(0 == lcm->handle());  

  return 0;
}
