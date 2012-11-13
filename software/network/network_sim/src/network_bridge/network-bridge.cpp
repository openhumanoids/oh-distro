#include "network-bridge.hpp"
#include <iostream>


using namespace std;
using boost::math::normal; // typedef provides default type is double.

static boost::minstd_rand generator(27u); // seed
  boost::uniform_real<> uni_dist(0,1);
  boost::variate_generator<boost::minstd_rand&, boost::uniform_real<> > uni(generator, uni_dist);


network_bridge::network_bridge(lcm_t* publish_lcm,lcm_t* subscribe_lcm):
      publish_lcm_(publish_lcm),subscribe_lcm_(subscribe_lcm)  {

  bot_core_pose_t_subscribe(subscribe_lcm_, "POSE",
      network_bridge::pose_handler_aux, this);
  //bot_core_pose_t_subscribe(subscribe_lcm_, "CAMERA_STATE",
  //    network_bridge::pose_handler_aux, this);
  
  bot_core_image_t_subscribe(subscribe_lcm_, "CAMERALEFT",
      network_bridge::image_handler_aux, this);
  drc_robot_state_t_subscribe(subscribe_lcm_, "EST_ROBOT_STATE",
      network_bridge::robot_state_handler_aux, this);
  drc_heightmap_t_subscribe(subscribe_lcm_, "HEIGHT_MAP",
      network_bridge::heightmap_handler_aux, this);
  
  // channel, max_freq, last_utime
  resendlist_.push_back( Resend("CAMERALEFT", 1.0, 0) );
  resendlist_.push_back( Resend("POSE", 1.0, 0) );
  resendlist_.push_back( Resend("EST_ROBOT_STATE", 0.25, 0) );
  resendlist_.push_back( Resend("HEIGHT_MAP", 1.0, 0) );
  
}


int network_bridge::determine_resend_from_list(std::string channel, int64_t msg_utime){
 for (size_t i=0; i < resendlist_.size() ; i++){
   if ( resendlist_[i].channel.compare(channel) == 0){
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

void network_bridge::image_handler(const bot_core_image_t *msg,const char* channel){
  cout << "i";
  if (determine_resend_from_list(channel,msg->utime)){
    cout << "I\n"; 
    bot_core_image_t_publish(publish_lcm_, channel, msg);
  }
    
  /*
  if (uni() > 0.5){
    //cout << msg->utime << " RS passed\n";
    cout << msg->utime << " " << channel << " passed\n";
    bot_core_image_t_publish(publish_lcm_, channel, msg);
  }else{
    cout << msg->utime << " " << channel << " dropped\n";
    //cout << msg->utime << " RS dropped\n";
  } */
}


void network_bridge::robot_state_handler(const drc_robot_state_t *msg,const char* channel){
  cout << "s";
  if (determine_resend_from_list(channel,msg->utime)){
    cout << "S\n"; 
    drc_robot_state_t_publish(publish_lcm_, channel, msg);
  }
}

void network_bridge::pose_handler(const bot_core_pose_t *msg,const char* channel){
  cout << "p";
  if (determine_resend_from_list(channel,msg->utime)){
    cout << "P\n";
    bot_core_pose_t_publish(publish_lcm_, channel, msg);
  }  
}

void network_bridge::heightmap_handler(const drc_heightmap_t *msg,const char* channel){
  cout << "h";
  if (determine_resend_from_list(channel,msg->utime)){
    cout << "H\n";
    drc_heightmap_t_publish(publish_lcm_, channel, msg);
  }  
}

int
main(int argc, char ** argv)
{
  std::cout << "network_bridge\n";
  lcm_t * lcm_publish = lcm_create("udpm://239.255.12.68:1268?ttl=1");
  lcm_t * lcm_subscribe = lcm_create(NULL);//"udpm://239.255.12.67:1267?ttl=1");
  network_bridge app(lcm_publish, lcm_subscribe);

  while(1)
    lcm_handle(lcm_subscribe);

  lcm_destroy(lcm_subscribe);
  lcm_destroy(lcm_publish);
  return 0;
}
