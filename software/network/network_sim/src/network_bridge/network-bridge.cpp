#include <boost/thread.hpp>
#include <lcm/lcm.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/drc_lcmtypes.h>
#include <ConciseArgs>
using namespace std;

struct Resend{
  Resend(std::string channel, double max_freq, int64_t last_utime):
    channel(channel), max_freq(max_freq), last_utime(last_utime) {}
  std::string channel; // .. LCM channel
  double max_freq; // max freq of transmission
  int64_t last_utime; // last utime of transmission
};

///////////////////////////////////////////////////////////////
class KMCLApp{
  public:
    KMCLApp(lcm_t* publish_lcm, lcm_t* subscribe_lcm,
	 std::string config_fname);
    
    ~KMCLApp(){
    }
    
    lcm_t* publish_lcm;
    lcm_t* subscribe_lcm;
    
    void addResend( Resend resent_in ){  resendlist_.push_back(resent_in);    }
    bool determine_resend_from_list(std::string channel, int64_t msg_utime);    

private:
    std::string config_fname;
    std::vector<Resend> resendlist_;    
};  

KMCLApp::KMCLApp(lcm_t* publish_lcm,lcm_t* subscribe_lcm,
      std::string config_fname):
      publish_lcm(publish_lcm), subscribe_lcm(subscribe_lcm),
      config_fname(config_fname){
  cout <<"Config has been read\n";
}

bool KMCLApp::determine_resend_from_list(std::string channel, int64_t msg_utime){
 for (size_t i=0; i < resendlist_.size() ; i++){
   if ( resendlist_[i].channel.compare(channel) == 0){
     if (resendlist_[i].max_freq ==0){
       cout << "always send" << channel << "\n";
       return true;
     }
     double elapsed_time = (msg_utime - resendlist_[i].last_utime)/1E6;
     if (elapsed_time > 1/resendlist_[i].max_freq){
       resendlist_[i].last_utime = msg_utime;
       return true;
     }else{
       return false;
     }
   }
 }
 return false;
}

void heightmap_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                const drc_heightmap_t* msg, void* user_data){
  KMCLApp* app = (KMCLApp*) user_data;
  if (app->determine_resend_from_list(channel,msg->utime)){   drc_heightmap_t_publish(app->publish_lcm, channel, msg); }
}

void image_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                const bot_core_image_t* msg, void* user_data){
  KMCLApp* app = (KMCLApp*) user_data;
  if (app->determine_resend_from_list(channel,msg->utime)){   bot_core_image_t_publish(app->publish_lcm, channel, msg); }
}

void pose_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                const bot_core_pose_t* msg, void* user_data){
  KMCLApp* app = (KMCLApp*) user_data;
  if (app->determine_resend_from_list(channel,msg->utime)){   bot_core_pose_t_publish(app->publish_lcm, channel, msg); }
}

void map_params_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                const drc_map_params_t* msg, void* user_data){
  KMCLApp* app = (KMCLApp*) user_data;
  if (app->determine_resend_from_list(channel,msg->utime)){   drc_map_params_t_publish(app->publish_lcm, channel, msg); }
}

void robot_state_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                const drc_robot_state_t* msg, void* user_data){
  KMCLApp* app = (KMCLApp*) user_data;
  if (app->determine_resend_from_list(channel,msg->utime)){   drc_robot_state_t_publish(app->publish_lcm, channel, msg); }
}

void nav_goal_timed_handler(const lcm_recv_buf_t* rbuf, const char* channel,
                                const drc_nav_goal_timed_t* msg, void* user_data){
  KMCLApp* app = (KMCLApp*) user_data;
  if (app->determine_resend_from_list(channel,msg->utime)){   drc_nav_goal_timed_t_publish(app->publish_lcm, channel, msg); }
}


void robot2base() { 
  lcm_t * publish_lcm= lcm_create("udpm://239.255.12.68:1268?ttl=1");
  lcm_t * subscribe_lcm= lcm_create(NULL);//"udpm://239.255.12.67:1267?ttl=1");  
  string config_fname="add_config_later";
  KMCLApp* app= new KMCLApp(publish_lcm,subscribe_lcm,config_fname);
  drc_heightmap_t_subscribe(subscribe_lcm, "HEIGHT_MAP",heightmap_handler, app);  
  bot_core_image_t_subscribe(subscribe_lcm, "CAMERALEFT",image_handler, app);  
  bot_core_pose_t_subscribe(subscribe_lcm, "POSE", pose_handler, app);
  drc_robot_state_t_subscribe(subscribe_lcm, "EST_ROBOT_STATE", robot_state_handler, app);
  
  // channel, max_freq, last_utime
  app->addResend( Resend("HEIGHT_MAP", 1.0, 0) );
  app->addResend( Resend("CAMERALEFT", 1.0, 0) );
  app->addResend( Resend("POSE", 1.0, 0) );
  app->addResend( Resend("EST_ROBOT_STATE", 0.25, 0) );
  
  while (1)
      lcm_handle(subscribe_lcm);  
}

void base2robot() { 
  lcm_t * publish_lcm= lcm_create(NULL);//"udpm://239.255.12.67:1267?ttl=1");  
  lcm_t * subscribe_lcm= lcm_create("udpm://239.255.12.68:1268?ttl=1");
  string config_fname="add_config_later";
  KMCLApp* app= new KMCLApp(publish_lcm,subscribe_lcm,config_fname);
  drc_map_params_t_subscribe(subscribe_lcm, "MAP_CREATE",map_params_handler, app);      
  drc_nav_goal_timed_t_subscribe(subscribe_lcm, "NAV_GOAL_TIMED",nav_goal_timed_handler, app);      
  
  // channel, max_freq, last_utime
  app->addResend( Resend("MAP_CREATE", 0.0, 0) );
  app->addResend( Resend("NAV_GOAL_TIMED", 0.0, 0) );
  
  while (1)
      lcm_handle(subscribe_lcm);
}

int main (int argc, char ** argv) {
    using namespace boost; 
    thread thread_1 = thread(robot2base);
    thread thread_2 = thread(base2robot);
    thread_2.join();
    thread_1.join();
    return 0;
}
