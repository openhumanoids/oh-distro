// mfallon sept 2012

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <lcmtypes/bot_param_update_t.h>
#include "lcmgl_teleop.hpp"

using namespace std;


lcmgl_teleop::lcmgl_teleop(lcm_t* publish_lcm,lcm_t* subscribe_lcm):
          publish_lcm_(publish_lcm),
          subscribe_lcm_(subscribe_lcm){
  lcmgl_ = bot_lcmgl_init(publish_lcm, "lcmgl-teleop");

  viconstructs_vicon_t_subscribe(subscribe_lcm, "drc_vicon",
      lcmgl_teleop::vicon_handler_aux, this);

}


void lcmgl_teleop::vicon_handler(const viconstructs_vicon_t *msg){
  cout << "got msg\n";

  bot_lcmgl_push_matrix(lcmgl_);
  //bot_lcmgl_translated(lcmgl_, 0, 70, 0);  // example offset
  bot_lcmgl_point_size(lcmgl_, 1.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 0, 0, 1); // Blue
  for (size_t i=0; i < msg->models[0].nummarkers; ++i) {
    double xyz[3];
    xyz[0] = msg->models[0].markers[i].xyz.x/1000.0;
    xyz[1] = msg->models[0].markers[i].xyz.y/1000.0;
    xyz[2] = msg->models[0].markers[i].xyz.z/1000.0;
    cout << xyz[0] <<"|"<< xyz[1] <<"|"<< xyz[2] <<"\n"; 
    bot_lcmgl_vertex3f(lcmgl_, xyz[0], xyz[1], xyz[2]);
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);
  bot_lcmgl_switch_buffer(lcmgl_);  


}


int lcmgl_teleop::do_some_lcmgl(int argc, char **argv){

  // 2. Create and publish 100 points to LCMGL:
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_translated(lcmgl_, 0, 70, 0);  // example offset
  bot_lcmgl_point_size(lcmgl_, 1.5f);
  bot_lcmgl_begin(lcmgl_, GL_POINTS);  // render as points
  bot_lcmgl_color3f(lcmgl_, 0, 0, 1); // Blue
  for (size_t i=0; i < 100; ++i) {
    bot_lcmgl_vertex3f(lcmgl_, i*10,  i*10, i*10); // 100 points in line
  }
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);

  // 2. Create and publish 3 connected lines to LCMGL:
  bot_lcmgl_push_matrix(lcmgl_);
  bot_lcmgl_translated(lcmgl_, 0, 40, 0);  // example offset
  bot_lcmgl_point_size(lcmgl_, 1.5f);
  bot_lcmgl_begin(lcmgl_, GL_LINE_STRIP);  // render as points
  bot_lcmgl_color3f(lcmgl_, 1, 0, 0); // Red
  bot_lcmgl_vertex3f(lcmgl_, 10,  20, 30);
  bot_lcmgl_vertex3f(lcmgl_, 60,  90, 100);
  bot_lcmgl_vertex3f(lcmgl_, -50,  20, 120);
  bot_lcmgl_vertex3f(lcmgl_, 50,  20, -120);
  bot_lcmgl_end(lcmgl_);
  bot_lcmgl_pop_matrix(lcmgl_);
  bot_lcmgl_switch_buffer(lcmgl_);  


  return 1;
}

int main(int argc, char ** argv) {
  lcm_t * lcm;
  lcm = lcm_create(NULL);
  lcmgl_teleop app(lcm,lcm);
  int status =  app.do_some_lcmgl(argc, argv);

  while(1)
    lcm_handle(lcm);

  return 0;
}
