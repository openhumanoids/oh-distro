#include <iostream>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/assign/std/vector.hpp>
#include <lcm/lcm-cpp.hpp>

#include <pointcloud_tools/pointcloud_math.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include <ConciseArgs>


using namespace boost;
using namespace boost::assign;

using namespace std;


#define verbose_txt 0
// 0 means publish nothing
// 1 means publish very important only [runs without issue]
// 2 means publish important
// 3 means publish all
#define verbose_lcm 3 // was 0


class joints2frames{
  public:
    joints2frames(boost::shared_ptr<lcm::LCM> &publish_lcm,
      int reset_period_, double resolution_);
    
    ~joints2frames(){
    }
    
  private:
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
    boost::shared_ptr<lcm::LCM> lcm_;
    Isometry3dTime poseT_;
    int64_t last_request_utime_;
    
    int reset_period_;
    double resolution_;
};    




joints2frames::joints2frames(boost::shared_ptr<lcm::LCM> &publish_lcm,
        int reset_period_, double resolution_):
          lcm_(publish_lcm), poseT_(0, Eigen::Isometry3d::Identity()),
          reset_period_(reset_period_),
          resolution_(resolution_){

  
  last_request_utime_=0;
  lcm_->subscribe("EST_ROBOT_STATE",&joints2frames::robot_state_handler,this);  
}


void joints2frames::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){

  // NB: reset is based on clock time not on 
  // TODO: just use the drc_clock_t message in future:
  

  
  int64_t reset_period_utime = (int64_t) reset_period_*1E6;
  if (msg->utime - last_request_utime_ > reset_period_utime ){
    std::cout << "reset and wait: "<< reset_period_ <<" seconds\n";
        
    drc::map_params_t msgout;
    msgout.utime = msg->utime;
    msgout.message_id = 0;
    msgout.map_id = -1;
    msgout.resolution = resolution_;
    msgout.dimensions[0] = 40;//10;
    msgout.dimensions[1] = 40;//10;
    msgout.dimensions[2] = 40;//10;

    msgout.transform_to_local.translation.x = msg->origin_position.translation.x;
    msgout.transform_to_local.translation.y = msg->origin_position.translation.y;
    msgout.transform_to_local.translation.z = msg->origin_position.translation.z;
    msgout.transform_to_local.rotation.x = 0;
    msgout.transform_to_local.rotation.y = 0;
    msgout.transform_to_local.rotation.z = 0;
    msgout.transform_to_local.rotation.w = 1; // to keep world aligned  

    lcm_->publish("MAP_CREATE", &msgout);

    last_request_utime_ = msg->utime;
  }
}




int main(const int argc, const char** argv) {
  int reset_period=5;
  double resolution =0.05;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(reset_period, "r", "reset_period", "reset_period - in seconds");
  opt.add(resolution, "o", "resolution", "octomap_resolution");
  opt.parse();
  cout << "reset_period: " << reset_period << "\n";
  cout << "resolution: " << resolution << "\n";
  
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;  
  
  joints2frames app(lcm,reset_period, resolution);
  while(0 == lcm->handle());
  return 0;
  
/*  
  while (1==1){
    sleep(2);
    std::cout << "reset\n";
    
  BotViewHandler *vhandler = self->viewer->view_handler;

  drc_map_params_t msgout;
  msgout.utime = self->robot_utime;
  msgout.message_id = 0;
  msgout.map_id = -1;
  msgout.resolution = 0.02;
  msgout.dimensions[0] = 40;//10;
  msgout.dimensions[1] = 40;//10;
  msgout.dimensions[2] = 40;//10;
  
  msgout.transform_to_local.translation.x = self->robot_pos[0];
  msgout.transform_to_local.translation.y = self->robot_pos[1];
  msgout.transform_to_local.translation.z = self->robot_pos[2];
  msgout.transform_to_local.rotation.x = 0;
  msgout.transform_to_local.rotation.y = 0;
  msgout.transform_to_local.rotation.z = 0;
  msgout.transform_to_local.rotation.w = 1; // to keep world aligned  

  drc_map_params_t_publish(self->lc,"MAP_CREATE",&msgout);
  bot_viewer_set_status_bar_message(self->viewer, "Sent MAP_CREATE");    
    
  }
  return 0;*/
}
