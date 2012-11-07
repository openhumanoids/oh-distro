// mfallon sept 2012

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <getopt.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <path_util/path_util.h>
#include <lcmtypes/bot_param_update_t.h>

#include <lcmtypes/drc_robot_state_t.h>


using namespace std;

lcm_t* publish_lcm;
drc_robot_state_t * buffer_msg;

void on_est_state(const lcm_recv_buf_t *rbuf, const char *channel,
    const drc_robot_state_t *msg, void *user_data){
//  KMCLApp* kmclapp = (KMCLApp*) user_data;
  cout << "INFO: reinit state rec'ed\n";

  buffer_msg= drc_robot_state_t_copy(msg);
  return;
}

void on_camstate(const lcm_recv_buf_t *rbuf, const char *channel,
    const bot_core_pose_t *msg, void *user_data){
//  KMCLApp* kmclapp = (KMCLApp*) user_data;
  cout << "INFO: pose state rec'ed\n";

  buffer_msg->utime = msg->utime;
  drc_robot_state_t_publish( publish_lcm  , "TRUE_ROBOT_STATE", buffer_msg);
  return;
}

int main(int argc, char **argv){
  
  // 1. Read a log from file and grab the last state message:
  string log_fname=argv[1];// "/home/mfallon/Desktop/electric_robot_log/lcmlog-2012-11-04.00";
  string begin_timestamp="0";
  publish_lcm=lcm_create(NULL);
  lcm_t* subscribe_lcm;
  char *endptr = NULL;
  string lcmurl = string("file://") + log_fname + string("?speed=0&start_timestamp=") +  begin_timestamp; //1307727077792229");
  subscribe_lcm = lcm_create(lcmurl.c_str());
  if(!subscribe_lcm){
    fprintf(stderr, "Unable to initialize LCM\n");
    return -1;
  }
  cout << "INFO: Reading LCM Logfile: " << lcmurl.c_str() << "\n";
  cout << "INFO: Finished initialization. Enter a number to start the particle filter: ";
  
  drc_robot_state_t_subscribe(subscribe_lcm, "EST_ROBOT_STATE", on_est_state, NULL);
  // go!
  while(0 == lcm_handle(subscribe_lcm));// && !shutdown_flag);


  // 2. There after list publish the state message as current:
  bot_core_pose_t_subscribe(publish_lcm, "CAMERA_STATE", on_camstate, NULL);
  while(0 == lcm_handle(publish_lcm));// && !shutdown_flag);

}
