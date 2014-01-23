#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp>


using namespace std;

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~App(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    pointcloud_vis* pc_vis_;
    
    void poseBDIHandler(const lcm::ReceiveBuffer* rbuf, 
                           const std::string& channel, const  bot_core::pose_t* msg);
    
    void viconHandler(const lcm::ReceiveBuffer* rbuf, 
                      const std::string& channel, const  bot_core::rigid_transform_t* msg);

    void doWork(Eigen::Isometry3d worldbdi_to_bdibody, int64_t utime);
    
    bool world_to_body_vicon_init_;
    bool world_to_body_bdi_init_;
    bool world_tf_init_;
    
    // This is determined TF between BDI and Vicon
    // think of it as the place where the BDI nav system was started in
    Eigen::Isometry3d worldbdi_to_worldvicon_;
    // This is where it actually placed it, we want to subtrack this and add worldbdi_to_worldvicon_
    Eigen::Isometry3d worldbdi_to_bdibody_zerotime_;
    // The vicon position at zerotime, the bdi motion since zerotime is applied to it
    Eigen::Isometry3d worldvicon_to_body_vicon_zerotime_;
    
};




App::App(boost::shared_ptr<lcm::LCM> &lcm_):
    lcm_(lcm_){

  lcm_->subscribe( "POSE_BDI" ,&App::poseBDIHandler,this);
  
  lcm_->subscribe("VICON_BODY",&App::viconHandler,this);  
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(10000,"BDI at init",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(10001,"BDI motion since init",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(10002,"BDI motion from vicon init",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(10003,"Vicon at init",5,1) );
  
  
  
  world_tf_init_ = false;
  world_to_body_bdi_init_ = false;
  world_to_body_vicon_init_ = false;
  
}

void App::poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  std::cout << "i got mail\n";

  Eigen::Isometry3d worldbdi_to_bdibody;
  worldbdi_to_bdibody.setIdentity();
  worldbdi_to_bdibody.translation()  << msg->pos[0], msg->pos[1], msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1], 
                                               msg->orientation[2], msg->orientation[3]);
  worldbdi_to_bdibody.rotate(quat); 

  if (!world_to_body_bdi_init_){
    std::cout << "initialize bdi\n";
    world_to_body_bdi_init_ = true;
  }  
  
  
  if (world_to_body_vicon_init_  &&  world_to_body_bdi_init_  ){
    doWork(worldbdi_to_bdibody, msg->utime);
  }else{
    std::cout << "not initialized, quitting\n";
  }
  
}

void App::doWork(Eigen::Isometry3d worldbdi_to_bdibody, int64_t utime){
  
  if (!world_tf_init_){
    std::cout << "have poseBDI and vicon, can init\n";
    worldbdi_to_worldvicon_ = worldbdi_to_bdibody.inverse() * worldvicon_to_body_vicon_zerotime_;
    worldbdi_to_bdibody_zerotime_ = worldbdi_to_bdibody;
    world_tf_init_ = true;
    
    // how much have we moved since we started:
    Isometry3dTime worldbdi_to_bdibody_zerotime_T(utime, worldbdi_to_bdibody_zerotime_);
    pc_vis_->pose_to_lcm_from_list(10000, worldbdi_to_bdibody_zerotime_T); // all joints in world frame

    // in vicon world where where we at start time
    Isometry3dTime worldvicon_to_body_vicon_zerotime_T(utime, worldvicon_to_body_vicon_zerotime_);
    pc_vis_->pose_to_lcm_from_list(10003, worldvicon_to_body_vicon_zerotime_T); // all joints in world frame
    
  }
  
  // how much have we moved since we started:
  Eigen::Isometry3d bdibody_init_to_bdibody = worldbdi_to_bdibody_zerotime_.inverse() * worldbdi_to_bdibody;
  Isometry3dTime bdibody_init_to_bdibody_T(utime, bdibody_init_to_bdibody);
  pc_vis_->pose_to_lcm_from_list(10001, bdibody_init_to_bdibody_T); // all joints in world frame
  
  // ... applied to the initial vicon
  Eigen::Isometry3d worldvicon_to_bodybdi = worldvicon_to_body_vicon_zerotime_ * bdibody_init_to_bdibody;
  Isometry3dTime worldvicon_to_bodybdi_T(utime, worldvicon_to_bodybdi);
  pc_vis_->pose_to_lcm_from_list(10002, worldvicon_to_bodybdi_T); // all joints in world frame
  
}

void App::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){
  std::cout << "i got femail\n";
  
  if (!world_to_body_vicon_init_){
    std::cout << "initialize vicon\n";
    worldvicon_to_body_vicon_zerotime_.setIdentity();
    worldvicon_to_body_vicon_zerotime_.translation()  << msg->trans[0], msg->trans[1] , msg->trans[2];
    Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], 
                                               msg->quat[2], msg->quat[3]);
    worldvicon_to_body_vicon_zerotime_.rotate(quat); 
    world_to_body_vicon_init_ = true;
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
