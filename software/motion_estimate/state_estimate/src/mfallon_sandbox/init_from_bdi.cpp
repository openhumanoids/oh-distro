#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <path_util/path_util.h>
#include <ConciseArgs>

#include "visualization/collections.hpp"


#include "kdl/tree.hpp"
#include <estimate/common_conversions.hpp>



using namespace std;



////////////////////////////////////////
struct CommandLineConfig
{
};


class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);
    
    ~App(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    BotParam* botparam_;
    bot::frames* frames_cpp_;
    BotFrames* frames_;
    
    /////////
    void poseBDIHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    
    const CommandLineConfig cl_cfg_;   
    pointcloud_vis* pc_vis_; 

    Eigen::Isometry3d prev_worldbdi_to_body_;
    int64_t prev_vicon_utime_;    
};


App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_):
    lcm_(lcm_), cl_cfg_(cl_cfg_){
      
  //if (cl_cfg_.param_file == ""){
  //   botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  //}else{
    std::string param_file = "drc_robot_02.cfg";            
    std::string param_file_full = std::string(getConfigPath()) +'/' + std::string(param_file);
    botparam_ = bot_param_new_from_file(param_file_full.c_str());
  //}
  // TODO: not sure what do do here ... what if i want the frames from the file?
  //frames_ = bot_frames_new(NULL, botparam_);
  frames_ = bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  frames_cpp_ = new bot::frames(frames_);  
  
  //  frames_cpp_ = new bot::frames( lcm_);

  lcm_->subscribe("POSE_BDI",&App::poseBDIHandler,this);  
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  pc_vis_->obj_cfg_list.push_back( obj_cfg(10032,"Vicon (v)",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(10033,"Vicon at init",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(10034,"Vicon (v all)",5,0) );
  pc_vis_->pose_collection_reset(10034,"Vicon (v all)");
  // 10035 - link
  
  prev_vicon_utime_ =0;
}



void App::poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  Eigen::Isometry3d worldbdi_to_body;
  worldbdi_to_body.setIdentity();
  worldbdi_to_body.translation()  << msg->pos[0], msg->pos[1] , msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1], 
                                            msg->orientation[2], msg->orientation[3]);
  worldbdi_to_body.rotate(quat); 

  Eigen::Isometry3d body_to_frontplate;
  frames_cpp_->get_trans_with_utime( "frontplate_vicon" ,"body_vicon" , msg->utime, body_to_frontplate);    
  Eigen::Isometry3d worldbdi_to_frontplate = worldbdi_to_body * body_to_frontplate;
  

  bot_core::rigid_transform_t pose_msg;
  pose_msg.utime =   msg->utime;
  pose_msg.trans[0] = worldbdi_to_frontplate.translation().x();
  pose_msg.trans[1] = worldbdi_to_frontplate.translation().y();
  pose_msg.trans[2] = worldbdi_to_frontplate.translation().z();  
  Eigen::Quaterniond r_x(worldbdi_to_frontplate.rotation());
  pose_msg.quat[0] =  r_x.w();  
  pose_msg.quat[1] =  r_x.x();  
  pose_msg.quat[2] =  r_x.y();  
  pose_msg.quat[3] =  r_x.z();  
  lcm_->publish( "VICON_FRONTPLATE" , &pose_msg);

  
  prev_vicon_utime_ = msg->utime;  
}



int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
//  opt.parse();  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App app(lcm, cl_cfg);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}