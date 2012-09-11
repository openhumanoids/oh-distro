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
}


void network_bridge::pose_handler(const bot_core_pose_t *msg){
  if (uni() > 0.5){
    cout << msg->utime << " POSE passed\n";
    bot_core_pose_t_publish(publish_lcm_, "POSE",
                            msg);
  }else{
    cout << msg->utime << " POSE dropped\n";
  } 
}



int
main(int argc, char ** argv)
{
  std::cout << "network_bridge\n";
  lcm_t * lcm_publish = lcm_create("udpm://239.255.12.68:1268?ttl=1");
  lcm_t * lcm_subscribe = lcm_create("udpm://239.255.12.67:1267?ttl=1");
  network_bridge app(lcm_publish, lcm_subscribe);

  while(1)
    lcm_handle(lcm_subscribe);

  lcm_destroy(lcm_subscribe);
  lcm_destroy(lcm_subscribe);
  return 0;
}
