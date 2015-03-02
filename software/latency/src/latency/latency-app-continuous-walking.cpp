#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc/atlas_state_t.hpp"
#include "lcmtypes/drc/atlas_command_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/utime_two_t.hpp"
#include "lcmtypes/drc/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/drc/double_array_t.hpp"
#include "lcmtypes/drc/map_request_t.hpp"
#include "lcmtypes/drc/map_image_t.hpp"
#include "lcmtypes/drc/foot_contact_estimate_t.hpp"
#include "lcmtypes/bot_core/pose_t.hpp"
#include "lcmtypes/bot_core/images_t.hpp"

#include "lcmtypes/mav_estimator.hpp"

#include <latency/latency.hpp>

#include <ConciseArgs>
using namespace std;

class App
{
public:
  App(boost::shared_ptr<lcm::LCM> &_lcm, int period_);
  ~App() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  void handleAtlasStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::atlas_state_t * msg);

  void handleCamera(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t * msg);
  void handleCameraFiltered(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t * msg);
  void handleCameraFused(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t * msg);

  void handleMapRequest(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::map_request_t * msg);
  void handleMapDepth(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::map_image_t * msg);

  void handleFootcontact(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::foot_contact_estimate_t * msg);

  void handleFootstepPlanRequest(const lcm::ReceiveBuffer* rbuf, const std::string& chan);
  void handleFootstepPlan(const lcm::ReceiveBuffer* rbuf, const std::string& chan);

  int period_;
  
private:
  std::vector<Latency*> lats_;
  
  std::vector <float> lat_time_;
  std::vector <float> lat_msgs_;
  int counter_;

  bool measure_gpf_;
};

App::App(boost::shared_ptr<lcm::LCM> &_lcm, int period_):
    _lcm(_lcm),period_(period_){


  Latency* a_lat0 = new Latency(period_);
  lats_.push_back(a_lat0) ; //
  Latency* a_lat1 = new Latency(period_);
  lats_.push_back(a_lat1) ;
  Latency* a_lat2 = new Latency(period_);
  lats_.push_back(a_lat2) ;
  Latency* a_lat3 = new Latency(period_);
  lats_.push_back(a_lat3) ;
  
  lat_time_ = {0.0, 0.0, 0.0, 0.0};
  lat_msgs_ = {0.0, 0.0, 0.0, 0.0};
  
  _lcm->subscribe("ATLAS_STATE", &App::handleAtlasStateMsg, this);
  _lcm->subscribe("CAMERA", &App::handleCamera, this);
  _lcm->subscribe("CAMERA_FILTERED", &App::handleCameraFiltered, this);
  _lcm->subscribe("CAMERA_FUSED", &App::handleCameraFused, this);

  _lcm->subscribe("MAP_DEPTH", &App::handleMapDepth, this);
  _lcm->subscribe("MAP_REQUEST", &App::handleMapRequest, this);

  _lcm->subscribe("FOOT_CONTACT_ESTIMATE_SLOW", &App::handleFootcontact, this);

  _lcm->subscribe("FOOTSTEP_PLAN_REQUEST", &App::handleFootstepPlanRequest, this);
  _lcm->subscribe("FOOTSTEP_PLAN_RESPONSE", &App::handleFootstepPlan, this);
  counter_=0;

}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


int64_t atlas_utime_msg;
int temp_counter=0;
void App::handleAtlasStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::atlas_state_t * msg){
  int64_t utime_now = _timestamp_now();
  lats_[0]->add_from(msg->utime,  utime_now);
//  std::cout << temp_counter << " " << msg->utime << "\n";

  temp_counter++;
  atlas_utime_msg = msg->utime;
}



void App::handleCamera(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::images_t * msg){
  // std::cout <<  (atlas_utime_msg - msg->utime)*1E-3 << " atlas to camera msg latency\n";

//  temp_counter=0;
//  std::cout << "                    c  "<< msg->utime<< "\n"; 

  lats_[1]->add_from(msg->utime, atlas_utime_msg );


  bool new_data = lats_[0]->add_to(msg->utime, atlas_utime_msg, "SYNC", lat_time_[0], lat_msgs_[0] );
  
  if (new_data){
    
    if (counter_% 10==0){
      std::cout << "AST-ERS | CAM-FIL    |   CAM-FUS     |  REQ-MAP "
                << "   ||   "
                << "AST-ERS | CAM-FIL  | CAM-FUS   |   REQ-MAP  \n";//   <msec|msg>\n";
    }
    
    std::cout.precision(5);
    std::cout.setf( std::ios::fixed, std:: ios::floatfield ); // floatfield set to fixed
    std::cout << lat_time_[0] << " | " << lat_time_[1] << "   |   " << lat_time_[2] << "   |   " << lat_time_[3] 
              << "   ||   "
              << lat_msgs_[0] << " | " << lat_msgs_[1] << "   |   " << lat_msgs_[2] << "   |   " << lat_msgs_[3] << "\n";
      

    drc::double_array_t msgout;
    msgout.utime = atlas_utime_msg;
    msgout.num_values = 2;
    msgout.values = { lat_time_[0],  lat_time_[1]};
    _lcm->publish( ("LATENCY") , &msgout);

    counter_++;
  }


}
void App::handleCameraFiltered(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t * msg)  {
  lats_[1]->add_to(msg->utime, atlas_utime_msg,  "FILT" , lat_time_[1], lat_msgs_[1]);
  lats_[2]->add_from(msg->utime, atlas_utime_msg );

//  std::cout << "                     f  "<< msg->utime<< "\n"; 

}
void App::handleCameraFused(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t * msg)  {
  lats_[2]->add_to(msg->utime, atlas_utime_msg,  "FUSE" , lat_time_[2], lat_msgs_[2]);


  std::cout << temp_counter << "                      k  "<< msg->utime<< "\n";
}

int64_t map_request_utime_now;
void App::handleMapRequest(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::map_request_t * msg){
  map_request_utime_now = _timestamp_now();
}
void App::handleMapDepth(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::map_image_t * msg){
  double dtime = (_timestamp_now() - map_request_utime_now)*1E-3 ;
  // std::cout << dtime <<" map server delay\n"; // neglibible delay
}


int64_t contact_change_utime_now;
float last_left_contact = 0;
float last_right_contact = 0;
void App::handleFootcontact(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::foot_contact_estimate_t * msg){
 
  if (last_left_contact != msg->left_contact){
    //std::cout << "left change\n";
    contact_change_utime_now = _timestamp_now();
  }
  if (last_right_contact != msg->right_contact){
    //std::cout << "right change\n";
    contact_change_utime_now = _timestamp_now();
  }


  last_left_contact = msg->left_contact;
  last_right_contact = msg->right_contact;
}


int64_t footstep_request_utime_now;
void App::handleFootstepPlanRequest(const lcm::ReceiveBuffer* rbuf, const std::string& chan){
  footstep_request_utime_now = _timestamp_now();

  double dtime = (_timestamp_now() - contact_change_utime_now)*1E-3 ;
  //std::cout << "                         " << dtime <<" segmentation planner delay\n";
}
void App::handleFootstepPlan(const lcm::ReceiveBuffer* rbuf, const std::string& chan){
  double dtime = (_timestamp_now() - footstep_request_utime_now)*1E-3 ;
  //std::cout << dtime <<" footstep planner delay\n";
}

int main (int argc, char ** argv){
  std::cout << "0:      ATLAS_STATE <-> EST_ROBOT_STATE\n";
  std::cout << "1:  ATLAS_IMU_BATCH <-> POSE_BODY\n";
  ConciseArgs parser(argc, argv, "latency-app");
  int period=200;
  parser.add(period, "p", "period", "Counting Period in samples");
  parser.parse();
  cout << "period is: " << period << " samples\n"; 
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  App app(lcm, period);
  cout << "App ready"<< endl;
  while(0 == lcm->handle());
  return 0;
}
