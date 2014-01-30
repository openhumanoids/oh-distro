#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <path_util/path_util.h>

#include "visualization/collections.hpp"

using namespace std;

class PoseTransformer{
  public:
    PoseTransformer(boost::shared_ptr<lcm::LCM> &lcm_, int pose_counter_);
    
    ~PoseTransformer(){
    }    
    void doWork(Eigen::Isometry3d worldest_to_estbody, int64_t utime, int pose_counter);
    
    bool world_to_viconbody_init_;
    bool world_to_estbody_init_;
    bool world_tf_init_;
    
    // This is determined TF between BDI and Vicon
    // think of it as the place where the BDI nav system was started in
    // This is where it actually placed it, 
    Eigen::Isometry3d worldest_to_estbody_zerotime_;
    // The vicon body position at zerotime, the bdi motion since zerotime is applied to it
    // to produce a estimate position that can be compared to VICON
    Eigen::Isometry3d worldvicon_to_viconbody_zerotime_;    
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    pointcloud_vis* pc_vis_;

    bool verbose_;
    int pose_counter_;
};

PoseTransformer::PoseTransformer(boost::shared_ptr<lcm::LCM> &lcm_, int pose_counter_):
    lcm_(lcm_), pose_counter_(pose_counter_){
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  if (pose_counter_==0){
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10000,"BDI at init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10001,"BDI motion since init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10002,"BDI (v)",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10003,"Vicon at init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10004,"BDI (v all)",5,0) );
  }else if (pose_counter_ ==1){
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10010,"MIT at init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10011,"MIT motion since init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10012,"MIT (v)",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10013,"Vicon at init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10014,"MIT (v all)",5,0) );
  }
  
  pc_vis_->pose_collection_reset(10004,"BDI (v all)");
  pc_vis_->pose_collection_reset(10014,"MIT (v all)");    
  
  world_to_viconbody_init_ = false;
  world_to_estbody_init_ = false;
  world_tf_init_ = false;
  verbose_ = false;  
}

void PoseTransformer::doWork(Eigen::Isometry3d worldest_to_estbody, int64_t utime, int pose_counter){
  
  if (!world_tf_init_){
    std::cout << "have estpose and vicon, can init "<< pose_counter <<"\n";
    worldest_to_estbody_zerotime_ = worldest_to_estbody;
    world_tf_init_ = true;
    
    if (verbose_){
      // how much have we moved since we started:
      Isometry3dTime worldest_to_estbody_zerotime_T(utime, worldest_to_estbody_zerotime_);
      pc_vis_->pose_to_lcm_from_list(10000 + pose_counter*10, worldest_to_estbody_zerotime_T); // all joints in world frame

      // in vicon world where where we at start time
      Isometry3dTime worldvicon_to_viconbody_zerotime_T(utime, worldvicon_to_viconbody_zerotime_);
      pc_vis_->pose_to_lcm_from_list(10003 + pose_counter*10, worldvicon_to_viconbody_zerotime_T); // all joints in world frame
    }
  }
  
  // how much have we moved since we started:
  Eigen::Isometry3d estbody_zerotime_to_estbody_current = worldest_to_estbody_zerotime_.inverse() * worldest_to_estbody;
  if (verbose_){
    Isometry3dTime estbody_zerotime_to_estbody_current_T(utime, estbody_zerotime_to_estbody_current);
    pc_vis_->pose_to_lcm_from_list(10001 + pose_counter*10, estbody_zerotime_to_estbody_current_T); // all joints in world frame
  }
  
  // ... applied to the initial vicon
  Eigen::Isometry3d worldvicon_to_estbody = worldvicon_to_viconbody_zerotime_ * estbody_zerotime_to_estbody_current;
  Isometry3dTime worldvicon_to_estbody_T(utime, worldvicon_to_estbody);
  pc_vis_->pose_to_lcm_from_list(10002 + pose_counter*10, worldvicon_to_estbody_T); // all joints in world frame
  pc_vis_->pose_to_lcm_from_list(10004 + pose_counter*10, worldvicon_to_estbody_T); // all joints in world frame
}



////////////////////////////////////////
class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~App(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    BotParam* botparam_;
    bot::frames* frames_cpp_;
    BotFrames* frames_;
    
    void poseBDIHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    void poseMITHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    
    void viconHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::rigid_transform_t* msg);

    std::vector <PoseTransformer> pts_;    
};




App::App(boost::shared_ptr<lcm::LCM> &lcm_):
    lcm_(lcm_){
      
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

  lcm_->subscribe( "POSE_BDI" ,&App::poseBDIHandler,this);
  lcm_->subscribe( "POSE_BODY_ALT" ,&App::poseMITHandler,this);
  lcm_->subscribe("VICON_FRONTPLATE",&App::viconHandler,this);  
  
  PoseTransformer pt0_(lcm_, 0);
  PoseTransformer pt1_(lcm_, 1);

  pts_.push_back( pt0_ );  
  pts_.push_back( pt1_ );    
}




void App::poseMITHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  //std::cout << "i got mit mail\n";
  
  PoseTransformer* pt = &(pts_[1]);

  Eigen::Isometry3d worldest_to_estbody;
  worldest_to_estbody.setIdentity();
  worldest_to_estbody.translation()  << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1],
                                               msg->orientation[2], msg->orientation[3]);
  worldest_to_estbody.rotate(quat);


  if (!pt->world_to_estbody_init_){
    std::cout << "initialize mit\n";
    pt->world_to_estbody_init_ = true;
  }    
  
  if (pt->world_to_viconbody_init_  &&  pt->world_to_estbody_init_  ){
    pt->doWork(worldest_to_estbody, msg->utime, 1);
  }else{
    std::cout << "not initialized, quitting mit\n";
  }
}


void App::poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  //std::cout << "i got mail\n";

  PoseTransformer* pt = &(pts_[0]);
  
  Eigen::Isometry3d worldest_to_estbody;
  worldest_to_estbody.setIdentity();
  worldest_to_estbody.translation()  << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1], 
                                               msg->orientation[2], msg->orientation[3]);
  worldest_to_estbody.rotate(quat); 

  if (!pt->world_to_estbody_init_){
    std::cout << "initialize bdi\n";
    pt->world_to_estbody_init_ = true;
  }  
  
  
  if (pt->world_to_viconbody_init_  &&  pt->world_to_estbody_init_  ){
    pt->doWork(worldest_to_estbody, msg->utime, 0);
  }else{
    std::cout << "not initialized, quitting\n";
  }
}



void App::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){
  for (size_t i=0; i < pts_.size() ; i++){
    PoseTransformer* pt = &(pts_[i]);
    
    if (!pt->world_to_viconbody_init_){
      std::cout << "initialize vicon\n";
      Eigen::Isometry3d worldvicon_to_frontplate_vicon_zerotime;
      worldvicon_to_frontplate_vicon_zerotime.setIdentity();
      worldvicon_to_frontplate_vicon_zerotime.translation()  << msg->trans[0], msg->trans[1] , msg->trans[2];
      Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], 
                                                msg->quat[2], msg->quat[3]);
      worldvicon_to_frontplate_vicon_zerotime.rotate(quat); 
      
      Eigen::Isometry3d frontplate_vicon_to_body_vicon;
      frames_cpp_->get_trans_with_utime( "body_vicon" , "frontplate_vicon", msg->utime, frontplate_vicon_to_body_vicon);    
      Eigen::Isometry3d worldvicon_to_body_vicon_zerotime = worldvicon_to_frontplate_vicon_zerotime * frontplate_vicon_to_body_vicon;
      
      pt->world_to_viconbody_init_ = true;
      pt->worldvicon_to_viconbody_zerotime_ = worldvicon_to_body_vicon_zerotime;
    }      
  }
}

int main(int argc, char ** argv) {
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App app(lcm);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}