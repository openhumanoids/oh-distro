// example usage:
// kmcl-ply-editor
//
//1. select pose
//2. look up all triangles within xm of pose
//3. find all unique colours inside that pose
//4. manual reassgn those colours to another r g b value from an array
//*********************************************************************
#include <sys/types.h>
#include <dirent.h>


#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <glib.h>
#include <glib-object.h>

#include <bot_core/bot_core.h>
#include <lcmtypes/drc_driving_control_cmd_t.h>
#include <lcmtypes/drc_driving_status_t.h>
//#include <common/globals.h>
//#include <lcmtypes/mrtypes.h>

#include <cstdio> 

#include <vector>
#include <lcmtypes/visualization.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"


#include <ncurses.h>
#include <wchar.h>

#include <iostream>
#include <sstream>
using namespace std;

#define COLOR_PLAIN 1
#define COLOR_TITLE 2
#define COLOR_ERROR 3
#define COLOR_WARN  4
#define STEP_TESTING 1

ros::Publisher hand_wheel_pub;
ros::Publisher hand_brake_pub;
ros::Publisher gas_pedal_pub;
ros::Publisher brake_pedal_pub;
ros::Publisher direction_pub;
ros::Publisher key_pub;

ros::Publisher exit_pub;
ros::Publisher enter_pub;

//http://gazebosim.org/wiki/Tutorials/drcsim/2.2/VRC_Plugin_DRC_Vehicle

typedef struct  {
  WINDOW *w;
  int width, height;

  lcm_t* lcm;

  GMainLoop * mainloop;
  guint timer_id;

  string robot_name;
  double hand_wheel;
  double hand_brake;
  double gas_pedal;
  double brake_pedal;

  int direction;
  int key;

} state_t;



static int publish_gas_pedal(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);

  if (s->gas_pedal >=0.09) {    s->gas_pedal = 0.09; }
  if (s->gas_pedal <=0.0) {    s->gas_pedal = 0.0; }

  std_msgs::Float64 msg;
  msg.data = s->gas_pedal;
  gas_pedal_pub.publish(msg);
  return 0; 
}

static int publish_hand_brake(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);

  if (s->hand_brake >=1.0) {    s->hand_brake = 1.0; }
  if (s->hand_brake <=0.0) {    s->hand_brake = 0.0; }

  std_msgs::Float64 msg;
  msg.data = s->hand_brake;
  hand_brake_pub.publish(msg);
  fprintf(stderr, "HB Called : %f\n", s->hand_brake);
  return 0; 
}

static int publish_hand_wheel(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);

  if (s->hand_wheel >=7.0) {    s->hand_wheel = 7.0; }
  if (s->hand_wheel <=-7.0) {    s->hand_wheel = -7.0; }

  std_msgs::Float64 msg;
  msg.data = s->hand_wheel;
  hand_wheel_pub.publish(msg);
  return 0; 
}



static int publish_brake_pedal(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);

  if (s->brake_pedal >=1.0) {    s->brake_pedal = 1.0; }
  if (s->brake_pedal <=0.0) {    s->brake_pedal = 0.0; }

  std_msgs::Float64 msg;
  msg.data = s->brake_pedal;
  brake_pedal_pub.publish(msg);
  return 0; 
}

/*
  The direction state and key state topics send an Int8 value. The direction state reports "1" for Forward, "0" for Neutral, and "-1" for Reverse. The key state reports "1" for On, "0" for Off, and "-1" for an error caused by turning the key to On when the direction switch is not in Neutral. Putting the direction switch back to neutral will restore the key state to "1". The key switch defaults to "On" and the direction to "Forward", but this may not be the case in future versions of the software or in the competition.

 */

static int publish_key(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);
  std_msgs::Int8 msg;
  msg.data = (int8_t) s->key;
  key_pub.publish(msg);
  return 0; 
}

//this is incremental - it needs to be added to the state 
static int publish_direction(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);
  std_msgs::Int8 msg;
  msg.data = s->direction;
  direction_pub.publish(msg);
  return 0; 
}

static int update_and_publish_gas_pedal(double new_val, state_t *s){
  if(new_val != s->gas_pedal){
    s->gas_pedal = new_val;
    return publish_gas_pedal(s);
  }
  return 0;
}

static int update_and_publish_hand_brake(double new_val, state_t *s){
  //if(new_val != s->hand_brake){
  s->hand_brake = new_val;
  return publish_hand_brake(s);
  //}
  //return 0;
}

/*
  static int update_to_and_publish_hand_wheel(double new_val, state_t *s){
  double delta = new_val - s->hand_wheel;
  
  if(delta >0){
    std_msgs::Float64 msg;

    msg.data = delta;
    hand_wheel_pub.publish(msg);

    //how long ?? 
    sleep(1);
    msg.data = 0;
    hand_wheel_pub.publish(msg);
    
    s->hand_wheel += delta;

    fprintf(stderr, "Delta : %f (deg)  Steering Angle : %f (deg) - %f (rad) \n", bot_to_degrees(delta), bot_to_degrees(s->hand_wheel), s->hand_wheel);    

    if (s->hand_wheel >=7.0) {    s->hand_wheel = 7.0; }
    if (s->hand_wheel <=-7.0) {    s->hand_wheel = -7.0; }
  }

  return 0;
}

static int update_and_publish_hand_wheel(double delta, state_t *s){
  if(delta >0){
    std_msgs::Float64 msg;
    msg.data = delta;
    hand_wheel_pub.publish(msg);

    //how long ?? 
    sleep(1);
    msg.data = 0;
    hand_wheel_pub.publish(msg);

    s->hand_wheel += delta;

    fprintf(stderr, "Delta : %f (deg)  Steering Angle : %f (deg) - %f (rad) \n", bot_to_degrees(delta), bot_to_degrees(s->hand_wheel), s->hand_wheel);

    if (s->hand_wheel >=7.0) {    s->hand_wheel = 7.0; }
    if (s->hand_wheel <=-7.0) {    s->hand_wheel = -7.0; }
  }

  return 0;
}
 */

static int update_and_publish_hand_wheel_delta(double delta, state_t *s){
  s->hand_wheel += delta;
  if (s->hand_wheel >=7.0) {    s->hand_wheel = 7.0; }
  if (s->hand_wheel <=-7.0) {    s->hand_wheel = -7.0; }

  std_msgs::Float64 msg;
  
  msg.data = s->hand_wheel;
  hand_wheel_pub.publish(msg);
  
  fprintf(stderr, "Delta : %f (deg)  Steering Angle : %f (deg) - %f (rad) \n", bot_to_degrees(delta), bot_to_degrees(s->hand_wheel), s->hand_wheel);    

  return 0;
}

static int update_and_publish_hand_wheel(double new_val, state_t *s){
  double delta = new_val - s->hand_wheel;
  s->hand_wheel = new_val;
  if (s->hand_wheel >=7.0) {    s->hand_wheel = 7.0; }
  if (s->hand_wheel <=-7.0) {    s->hand_wheel = -7.0; }
  std_msgs::Float64 msg;
  msg.data = s->hand_wheel;
  hand_wheel_pub.publish(msg);

  fprintf(stderr, "Delta : %f (deg)  Steering Angle : %f (deg) - %f (rad) \n", bot_to_degrees(delta), bot_to_degrees(s->hand_wheel), s->hand_wheel);

  return 0;
}

static int update_and_publish_brake_pedal(double new_val, state_t *s){
  if(new_val != s->brake_pedal){
    s->brake_pedal = new_val;
    return publish_brake_pedal(s);
  }
  return 0;
}

static int update_and_publish_key(int new_val, state_t *s){
  if(new_val != s->key){
    s->key = new_val;
    return publish_key(s);
  }
  return 0;
}

static int update_and_publish_direction(int new_val, state_t *s){
  if(new_val != s->direction){
    s->direction = new_val;
    return publish_direction(s);
  }
  return 0;
}

static int publish_enter(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);
  geometry_msgs::Pose msg;
  msg.position.x =0.0;
  msg.position.y =0.0;
  msg.position.z =0;
  msg.orientation.w =1.0;
  msg.orientation.x =0.0;
  msg.orientation.y =0.0;
  msg.orientation.z =0.0;
  enter_pub.publish(msg);
  
//rostopic pub --once /drc_world/robot_exit_car geometry_msgs/Pose '{position: {x: 0.5, y: -3.5, z: 0}, //orientation: {w: 0.707, x: 0, y: 0, z: 0.707}}'
  return 0; 
}
static int publish_exit(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);
  geometry_msgs::Pose msg;
  msg.position.x =0.5;
  msg.position.y =-3.5;
  msg.position.z =0;
  msg.orientation.w =0.707;
  msg.orientation.x =0.0;
  msg.orientation.y =0.0;
  msg.orientation.z =0.707;
  exit_pub.publish(msg);
  

//  std_msgs::Int8 msg;
//  msg.data = s->direction;
//  direction_pub.publish(msg);
  return 0; 
}

static int
repaint (state_t * s)
{
  WINDOW * w = s->w;
  clear();
  getmaxyx(w, s->height, s->width);
  color_set(COLOR_PLAIN, NULL);
  wmove(w, s->height/2,0);
  
  color_set(COLOR_PLAIN, NULL);
  wmove(w, 1, 0);
  wprintw(w, "        command     [ keys| range]",s->brake_pedal);
  wmove(w, 2, 0);
  wprintw(w, "%f brake_pedal[W  S | 0   1]",s->brake_pedal);
  wmove(w, 3, 0);
  wprintw(w, "%f hand_brake [Q  A | 0   1]",s->hand_brake);
  wmove(w, 4, 0);
  wprintw(w, "%f gas_pedal  [^ \\\/ | 0 0.9]",s->gas_pedal);
  wmove(w, 5, 0);
  wprintw(w, "%f hand_wheel [<  > | 0   7]",s->hand_wheel);
  wmove(w, 7, 0);
  wprintw(w, "%d direction  [234  |-1 0 1]",s->direction);
  wmove(w, 8, 0);
  wprintw(w, "%d key        [i  o | 0   1]",s->key);
  
  wmove(w, 10, 0);
  wprintw(w, " exit[k] enter[l]    allbrakesoff[e]");

  color_set(COLOR_TITLE, NULL);
  wrefresh (w);
  return 0;
}

void init_state(state_t *s){
  s->hand_wheel = 0;
  s->gas_pedal = 0;
  s->hand_brake = 0.01;
  s->brake_pedal = 0.0;
  s->key = 1.0;
  s->direction = 0.0;

  //publish these values 
  publish_hand_brake(s);
  publish_gas_pedal(s);
  update_and_publish_hand_wheel(0, s);
  publish_brake_pedal(s);
  publish_key(s);
  publish_direction(s);
}


void on_driving_cmd(const lcm_recv_buf_t *rbuf, const char * channel, const drc_driving_control_cmd_t * msg, void * user) {
  state_t *self = (state_t *) user;

  static int first = 1;
  if(first)
    init_state(self);
  
  first = 0;

  fprintf(stderr, "Command => Utime : %f\n", msg->utime/1.0e6);
  if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_START_CAR){
    fprintf(stderr, "Starting vehicle - this should be ignored - vehicle always on\n");
    //self->key = 1;
    //publish_key(user);
    self->direction = 0;
    publish_direction(self);
    update_and_publish_key(1, self);
    self->direction = 1;
    publish_direction(self);
  }
  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_SWITCH_OFF_ENGINE){
    fprintf(stderr, "Switching off engine - this should be ignored - vehicle always on\n");
    update_and_publish_key(0, self);
  } 
  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_GEAR_FORWARD){
    fprintf(stderr, "Putting gear forward\n");
    update_and_publish_direction(1, self);
  } 
  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_GEAR_NEUTRAL){
    fprintf(stderr, "Putting gear neutral\n");
    update_and_publish_direction(0, self);
  } 
  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_GEAR_REVERSE){
    fprintf(stderr, "Putting gear reverse\n");
    update_and_publish_direction(-1, self);
  } 
  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_DRIVE){
    fprintf(stderr, "Driving - turning wheel heading (deg): %f Throttle : %f Brake : %f\n", bot_to_degrees(msg->steering_angle), msg->throttle_value,  msg->brake_value);
    update_and_publish_hand_brake(0.0, self);
    update_and_publish_brake_pedal(msg->brake_value, self);
    update_and_publish_hand_wheel(msg->steering_angle, self);
    update_and_publish_gas_pedal(msg->throttle_value, self);
  } 
  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_DRIVE_DELTA_STEERING){
      fprintf(stderr, "Driving - turning wheel delta heading (deg): %f Throttle : %f Brake : %f \n", bot_to_degrees(msg->steering_angle), msg->throttle_value, msg->brake_value);
    update_and_publish_hand_brake(0.0, self);
    //update_and_publish_brake_pedal(0.0, self);
    update_and_publish_hand_wheel_delta(msg->steering_angle, self);
    update_and_publish_brake_pedal(msg->brake_value, self);
    update_and_publish_gas_pedal(msg->throttle_value, self);
  } 
  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_BRAKE){
    fprintf(stderr, "Applying brake %f\n", msg->brake_value); 
    //update_and_publish_hand_wheel(msg->steering_angle, self);
    update_and_publish_brake_pedal(msg->brake_value, self);
    update_and_publish_gas_pedal(0, self);
  } 
  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_E_STOP){
    fprintf(stderr, "E Stopping Vehicle \n");
    update_and_publish_brake_pedal(1.0, self);
  }
  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_HANDBRAKE_ON){
    fprintf(stderr, "Applying Handbrake \n");
    //update_and_publish_hand_brake(0.05, self);
    //update_and_publish_hand_brake(.3, self);
    update_and_publish_hand_brake(1.0, self);
    //update_and_publish_hand_brake(1.0, self);
    //update_and_publish_hand_brake(1.0, self);
  }
  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_HANDBRAKE_OFF){
    fprintf(stderr, "Removing Handbrake\n");
    update_and_publish_hand_brake(0.00, self);
  }

  else if(msg->type == DRC_DRIVING_CONTROL_CMD_T_TYPE_START_SEQUENCE){
    fprintf(stderr, "Initializing start sequence\n");
    self->hand_brake = 0.05;
    publish_hand_brake(self);
    sleep(1);
    self->hand_brake = 0.0;
    publish_hand_brake(self);
    self->brake_pedal = 0.0;
    publish_brake_pedal(self);
    self->direction = 0;
    publish_direction(self);
    sleep(1);
    self->direction = 1;
    publish_direction(self);
    //self->key = 0;
    //publish_key(self);
    //sleep(1);
    //self->key = 1;
    //publish_key(self);
    fprintf(stderr, "Done with the startup");
  }
}


static gboolean
on_timer (void * user)
{
  state_t* s = static_cast<state_t*>(user);
  //publish state
  drc_driving_status_t msg;
  msg.utime = bot_timestamp_now();
  msg.hand_wheel = s->hand_wheel;
  msg.hand_brake = s->hand_brake;
  msg.gas_pedal = s->gas_pedal;
  msg.brake_pedal = s->brake_pedal;
  msg.direction = s->direction;
  msg.key = s->key;

  drc_driving_status_t_publish(s->lcm, "DRC_DRIVING_STATUS", &msg);
  
  return TRUE;
}


int main(int argc, char *argv[])
{
  state_t* state = new state_t();
  state->lcm= lcm_create(NULL);

  state->robot_name = "drc_vehicle";
  ros::init(argc, argv, "keyboard_driving");
  ros::NodeHandle n;
  hand_wheel_pub = n.advertise<std_msgs::Float64>(state->robot_name +"/hand_wheel/cmd", 1000);
  hand_brake_pub = n.advertise<std_msgs::Float64>(state->robot_name +"/hand_brake/cmd", 1000);
  gas_pedal_pub = n.advertise<std_msgs::Float64>(state->robot_name +"/gas_pedal/cmd", 1000);
  brake_pedal_pub = n.advertise<std_msgs::Float64>(state->robot_name +"/brake_pedal/cmd", 1000);
  direction_pub = n.advertise<std_msgs::Int8>(state->robot_name +"/direction/cmd", 1000);
  key_pub = n.advertise<std_msgs::Int8>(state->robot_name +"/key/cmd", 1000);

  exit_pub = n.advertise<geometry_msgs::Pose>("/drc_world/robot_exit_car", 1000);
  enter_pub = n.advertise<geometry_msgs::Pose>("/drc_world/robot_enter_car", 1000);
  
  state->mainloop = g_main_loop_new (NULL, FALSE);
  bot_glib_mainloop_attach_lcm (state->lcm);
  bot_signal_pipe_glib_quit_on_kill (state->mainloop);

  //subscribe to control channel 
  drc_driving_control_cmd_t_subscribe(state->lcm, "DRC_DRIVING_COMMAND", on_driving_cmd, state);

  state->timer_id = g_timeout_add (100, on_timer, state);
  init_state(state);
  /* Watch stdin */
  /*GIOChannel * channel = g_io_channel_unix_new (0);
  g_io_add_watch (channel, G_IO_IN, on_input, state);
  state->timer_id = g_timeout_add (25, on_timer, state);

  state->w = initscr();
  start_color();
  cbreak();
  noecho();

  init_pair(COLOR_PLAIN, COLOR_WHITE, COLOR_BLACK);
  init_pair(COLOR_TITLE, COLOR_BLACK, COLOR_WHITE);
  init_pair(COLOR_WARN, COLOR_BLACK, COLOR_YELLOW);
  init_pair(COLOR_ERROR, COLOR_BLACK, COLOR_RED);*/

  g_main_loop_run (state->mainloop);
  
  endwin ();
  g_source_remove (state->timer_id);
  bot_glib_mainloop_detach_lcm (state->lcm);
  g_main_loop_unref (state->mainloop);
  state->mainloop = NULL;
  //globals_release_lcm (s->lc);
  free (state);
}
