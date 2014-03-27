#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <path_util/path_util.h>
#include <ConciseArgs>

#include "visualization/collections.hpp"


#include "kdl/tree.hpp"
#include <leg_estimate/common_conversions.hpp>



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
    pointcloud_vis* pc_vis_;
    int pose_counter_;
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;

    bool verbose_;
    int64_t prev_utime_;
    string base_string_;
};

PoseTransformer::PoseTransformer(boost::shared_ptr<lcm::LCM> &lcm_, int pose_counter_):
    lcm_(lcm_), pose_counter_(pose_counter_){
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  if (pose_counter_==0){
    base_string_ = "EST";
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10000,"EST at init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10001,"EST motion since init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10002,"EST (v)",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10003,"Vicon at init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10004,"EST (v all)",5,0) );
    // 10005 - link
    pc_vis_->pose_collection_reset(10004,"EST (v all)");
    pc_vis_->pose_collection_reset(10005,"EST (l)");    
  }else if (pose_counter_ ==1){
    base_string_ = "MIT";
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10010,"MIT at init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10011,"MIT motion since init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10012,"MIT (v)",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10013,"Vicon at init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10014,"MIT (v all)",5,0) );
    // 10015 - link
    pc_vis_->pose_collection_reset(10014,"MIT (v all)");    
    pc_vis_->pose_collection_reset(10015,"MIT (v all)");    
  }else if (pose_counter_ ==2){
    base_string_ = "BDI";
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10020,"BDI at init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10021,"BDI motion since init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10022,"BDI (v)",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10023,"Vicon at init",5,1) );
    pc_vis_->obj_cfg_list.push_back( obj_cfg(10024,"BDI (v all)",5,0) );
    // 10025 - link
    pc_vis_->pose_collection_reset(10024,"BDI (v all)"); 
    pc_vis_->pose_collection_reset(10025,"BDI (l)");    
  }
  
  
  world_to_viconbody_init_ = false;
  world_to_estbody_init_ = false;
  world_tf_init_ = false;
  verbose_ = false;  
  prev_utime_ = 0;
}

void PoseTransformer::doWork(Eigen::Isometry3d worldest_to_estbody, int64_t utime, int pose_counter){
  
  if (!world_tf_init_){
    std::cout << "have estpose and vicon, can init "<< pose_counter <<"\n";
    worldest_to_estbody_zerotime_ = worldest_to_estbody;
    world_tf_init_ = true;
    
    if (verbose_){
      // how much have we moved since we started:
      Isometry3dTime worldest_to_estbody_zerotime_T(utime, worldest_to_estbody_zerotime_);
      pc_vis_->pose_to_lcm_from_list(10000 + pose_counter*10, worldest_to_estbody_zerotime_T); 

      // in vicon world where where we at start time
      Isometry3dTime worldvicon_to_viconbody_zerotime_T(utime, worldvicon_to_viconbody_zerotime_);
      pc_vis_->pose_to_lcm_from_list(10003 + pose_counter*10, worldvicon_to_viconbody_zerotime_T); 
    }
  }
  
  // how much have we moved since we started:
  Eigen::Isometry3d estbody_zerotime_to_estbody_current = worldest_to_estbody_zerotime_.inverse() * worldest_to_estbody;
  if (verbose_){
    Isometry3dTime estbody_zerotime_to_estbody_current_T(utime, estbody_zerotime_to_estbody_current);
    pc_vis_->pose_to_lcm_from_list(10001 + pose_counter*10, estbody_zerotime_to_estbody_current_T); 
  }
  
  // ... applied to the initial vicon
  Eigen::Isometry3d worldvicon_to_estbody = worldvicon_to_viconbody_zerotime_ * estbody_zerotime_to_estbody_current;
  Isometry3dTime worldvicon_to_estbody_T(utime, worldvicon_to_estbody);
  pc_vis_->pose_to_lcm_from_list(10002 + pose_counter*10, worldvicon_to_estbody_T); 
  pc_vis_->pose_to_lcm_from_list(10004 + pose_counter*10, worldvicon_to_estbody_T); 
  
  link_data* ldata_single= new link_data(utime, (10004 + pose_counter*10), prev_utime_, 10004 + pose_counter*10, utime);
  link_cfg* lcfg = new link_cfg( (10005 + pose_counter*10), string(base_string_ + " (l)"), 0, false);
  pc_vis_->link_to_lcm(*lcfg, *ldata_single); 
  
  
  
  prev_utime_ = utime;
}



////////////////////////////////////////
struct CommandLineConfig
{
    bool use_pose_vicon;
    bool send_pose_vicon;
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
    
    void poseESTHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    void poseMITHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    void poseBDIHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    void processPose( const  bot_core::pose_t* msg, int pose_counter, std::string channel);
    
    /////////
    void viconPoseHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    void viconHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::rigid_transform_t* msg);
    void useVicon(Eigen::Isometry3d worldvicon_to_body_vicon, int64_t utime);
    
    std::vector <PoseTransformer> pts_;   
    const CommandLineConfig cl_cfg_;   
    pointcloud_vis* pc_vis_; 

    Eigen::Isometry3d prev_worldvicon_to_body_vicon_;
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

  lcm_->subscribe( "POSE_BODY" ,&App::poseESTHandler,this);
  lcm_->subscribe( "POSE_MIT" ,&App::poseMITHandler,this);
  lcm_->subscribe( "POSE_BDI" ,&App::poseBDIHandler,this);

  if (cl_cfg_.use_pose_vicon){
    lcm_->subscribe("POSE_VICON",&App::viconPoseHandler,this);  
  }else{
    lcm_->subscribe("VICON_FRONTPLATE",&App::viconHandler,this);  
  }
  
  
  PoseTransformer pt0_(lcm_, 0);
  PoseTransformer pt1_(lcm_, 1);
  PoseTransformer pt2_(lcm_, 2);
  pts_.push_back( pt0_ );  
  pts_.push_back( pt1_ );    
  pts_.push_back( pt2_ );    
  
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  pc_vis_->obj_cfg_list.push_back( obj_cfg(10032,"Vicon (v)",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(10033,"Vicon at init",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(10034,"Vicon (v all)",5,0) );
  pc_vis_->pose_collection_reset(10034,"Vicon (v all)");
  // 10035 - link
  
  prev_vicon_utime_ =0;
}



void App::processPose( const  bot_core::pose_t* msg, int pose_counter, std::string channel){
  PoseTransformer* pt = &(pts_[pose_counter]);

  Eigen::Isometry3d worldest_to_estbody;
  worldest_to_estbody.setIdentity();
  worldest_to_estbody.translation()  << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1],
                                               msg->orientation[2], msg->orientation[3]);
  worldest_to_estbody.rotate(quat);


  if (!pt->world_to_estbody_init_){
    std::cout << "initializing " << channel <<"\n";
    pt->world_to_estbody_init_ = true;
  }    
  
  if (pt->world_to_viconbody_init_  &&  pt->world_to_estbody_init_  ){
    pt->doWork(worldest_to_estbody, msg->utime, pose_counter);
  }else{
    std::cout << "not initialized "<< channel <<", quitting\n";
  }
}


void App::poseESTHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  processPose(msg, 0, channel);
}
void App::poseMITHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  processPose(msg, 1, channel);
}
void App::poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  processPose(msg, 2, channel);
}

void App::useVicon(Eigen::Isometry3d worldvicon_to_body_vicon, int64_t utime){
  if (prev_vicon_utime_ > utime){
    std::cout << "\n";
    std::cout << "out of order vicon detected, resetting\n";    
    pc_vis_->pose_collection_reset(10034 ,"Unused String");
    for (size_t i=0; i < pts_.size() ; i++){
      PoseTransformer* pt = &(pts_[i]);
      pt->world_to_viconbody_init_ = false;
      pt->world_to_estbody_init_ = false;
      pt->world_tf_init_ = false;      
      pt->pc_vis_->pose_collection_reset(10004 + i*10,"Unused String");
    }
  }
  
  // Publish Vicon
  Isometry3dTime worldvicon_to_body_vicon_T(utime, worldvicon_to_body_vicon);
  pc_vis_->pose_to_lcm_from_list(10032, worldvicon_to_body_vicon_T); 
  pc_vis_->pose_to_lcm_from_list(10034, worldvicon_to_body_vicon_T);     
  
  link_data* ldata_single= new link_data(utime, 10034, prev_vicon_utime_, 10034, utime);
  link_cfg* lcfg = new link_cfg( 10035, "Vicon (l)", 0, false);
  pc_vis_->link_to_lcm(*lcfg, *ldata_single);   
  
  for (size_t i=0; i < pts_.size() ; i++){
    PoseTransformer* pt = &(pts_[i]);
    
    if (!pt->world_to_viconbody_init_){
      std::cout << "initialize vicon\n";
      
      pt->world_to_viconbody_init_ = true;
      pt->worldvicon_to_viconbody_zerotime_ = worldvicon_to_body_vicon;
    }      
  }
  
}


void App::viconPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  Eigen::Isometry3d worldvicon_to_body_vicon;
  worldvicon_to_body_vicon.setIdentity();
  worldvicon_to_body_vicon.translation()  << msg->pos[0], msg->pos[1] , msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1], 
                                            msg->orientation[2], msg->orientation[3]);
  worldvicon_to_body_vicon.rotate(quat); 

  useVicon(worldvicon_to_body_vicon, msg->utime);
  prev_vicon_utime_ = msg->utime;  
}



void App::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){
  
  Eigen::Isometry3d worldvicon_to_frontplate_vicon;
  worldvicon_to_frontplate_vicon.setIdentity();
  worldvicon_to_frontplate_vicon.translation()  << msg->trans[0], msg->trans[1] , msg->trans[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], 
                                            msg->quat[2], msg->quat[3]);
  worldvicon_to_frontplate_vicon.rotate(quat); 
  Eigen::Isometry3d frontplate_vicon_to_body_vicon;
  frames_cpp_->get_trans_with_utime( "body_vicon" , "frontplate_vicon", msg->utime, frontplate_vicon_to_body_vicon);    
  Eigen::Isometry3d worldvicon_to_body_vicon = worldvicon_to_frontplate_vicon * frontplate_vicon_to_body_vicon;

  useVicon(worldvicon_to_body_vicon, msg->utime);  
  
  if (cl_cfg_.send_pose_vicon){
    bot_core::pose_t pose_vicon = getPoseAsBotPose(worldvicon_to_body_vicon, msg->utime);
    // determine the vicon body frame velocity:
    if (prev_vicon_utime_ > 0){
      // the delta transform between the previous and current 
      Eigen::Isometry3d delta_vicon = prev_worldvicon_to_body_vicon_.inverse() * worldvicon_to_body_vicon;
      double elapsed_time = ( (double) msg->utime -  prev_vicon_utime_)/1000000;
      pose_vicon.vel[0] = delta_vicon.translation().x() / elapsed_time;
      pose_vicon.vel[1] = delta_vicon.translation().y() / elapsed_time;
      pose_vicon.vel[2] = delta_vicon.translation().z() / elapsed_time;
    }  
    lcm_->publish( "POSE_VICON" , &pose_vicon);
    prev_worldvicon_to_body_vicon_ = worldvicon_to_body_vicon;
  }

  
  prev_vicon_utime_ = msg->utime;  
}

int main(int argc, char ** argv) {
  CommandLineConfig cl_cfg;
  cl_cfg.use_pose_vicon = false;
  cl_cfg.send_pose_vicon = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(cl_cfg.use_pose_vicon, "p", "use_pose_vicon","Will a POSE_VICON be published");
  opt.add(cl_cfg.send_pose_vicon, "s", "send_pose_vicon","Publish a POSE_VICON (if creaving FRONTPLATE");
  opt.parse();  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  App app(lcm, cl_cfg);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
