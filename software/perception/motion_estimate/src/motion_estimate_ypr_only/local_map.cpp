// LCM example program for interacting  with PCL data
// In this case it pushes the data back out to LCM in
// a different message type for which the "collections"
// renderer can view it in the LCM viewer
// mfallon aug 2012
#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>




#include "local_map.hpp"

using namespace std;





/////////////////////////////////////

local_map::local_map(lcm_t* publish_lcm):
          publish_lcm_(publish_lcm){

  // LCM:
  lcm_t* subscribe_lcm_ = publish_lcm_;

  bot_core_pose_t_subscribe(subscribe_lcm_, "POSE_HEAD_ORIENT",
      local_map::pose_handler_aux, this);

}

int counter =0;
void local_map::pose_handler(const bot_core_pose_t *msg){
  bot_core_pose_t msgout;
  counter++;
  if (counter%200 ==0){
    std::cout << counter << " POSE_HEAD\n";
  }  

  bot_core_pose_t_publish( publish_lcm_  , "POSE_HEAD", msg);
}


int
main(int argc, char ** argv)
{
  lcm_t * lcm;
  lcm = lcm_create(NULL);
  local_map app(lcm);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}

