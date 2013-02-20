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

typedef struct  {
  WINDOW *w;
  int width, height;

  lcm_t* publish_lcm;

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

static int publish_key(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);
  std_msgs::Int8 msg;
  msg.data = (int8_t) s->key;
  key_pub.publish(msg);
  return 0; 
}

static int publish_direction(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);
  std_msgs::Int8 msg;
  msg.data = s->direction;
  direction_pub.publish(msg);
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


static gboolean
on_input (GIOChannel * source, GIOCondition cond, gpointer data)
{
  state_t* s = static_cast<state_t*>(data);
  WINDOW * w = s->w;
  int c = getch();
    
  double d_hand_wheel =0.3;
  double d_hand_brake =0.05;
  double d_gas_pedal =0.001;
  double d_brake_pedal =0.05;
  switch (c)
  {
    case 65: // up arrow: +gas
      s->gas_pedal += d_gas_pedal ;
      publish_gas_pedal(s);
      break;
    case 66: // down arrow: -gas
      s->gas_pedal -= d_gas_pedal ;
      publish_gas_pedal(s);
      break;
    case 68: // left arrow: steer left
      s->hand_wheel += d_hand_wheel ;
      publish_hand_wheel(s);
      break;
    case 67: // right arrow: steer right
      s->hand_wheel -= d_hand_wheel ;
      publish_hand_wheel(s);
      break;
    case 'q':
      s->hand_brake -= d_hand_brake ;
      publish_hand_brake(s);
      break;
    case 'a':
      s->hand_brake += d_hand_brake ;
      publish_hand_brake(s);
      break;
    case 'w':
      s->brake_pedal -= d_brake_pedal ;
      publish_brake_pedal(s);
      break;
    case 's':
      s->brake_pedal += d_brake_pedal ;
      publish_brake_pedal(s);
      break;
    case 'e':
      s->hand_brake= 0.0 ;
      publish_hand_brake(s);
      s->brake_pedal= 0.0 ;
      publish_brake_pedal(s);
      break;
    case '1':
      s->direction = -1 ;
      publish_direction(s);
      break;
    case '2':
      s->direction = 0 ;
      publish_direction(s);
      break;
    case '3':
      s->direction = 1 ;
      publish_direction(s);
      break;
    case 'i':
      s->key = 1 ;
      publish_key(s);
      break;
    case 'o':
      s->key = 0;
      publish_key(s);
      break;
    case 'k':
      publish_exit(s);
      break;
    case 'l':
      publish_enter(s);
      break;
    case ' ':
      s->hand_brake = 1.0 ;
      publish_hand_brake(s);
      s->brake_pedal = 1.0 ;
      publish_brake_pedal(s);
      s->hand_wheel = 0.0 ;
      publish_hand_wheel(s);
      s->gas_pedal = 0.0 ;
      publish_gas_pedal(s);
  }
    
  repaint (s);	
  return TRUE;
}

static gboolean
on_timer (void * user)
{
  state_t* s = static_cast<state_t*>(user);
  repaint (s);
  return TRUE;
}

int main(int argc, char *argv[])
{
  state_t* state = new state_t();
  state->publish_lcm= lcm_create(NULL);

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
  bot_glib_mainloop_attach_lcm (state->publish_lcm);
  bot_signal_pipe_glib_quit_on_kill (state->mainloop);

  /* Watch stdin */
  GIOChannel * channel = g_io_channel_unix_new (0);
  g_io_add_watch (channel, G_IO_IN, on_input, state);
  state->timer_id = g_timeout_add (25, on_timer, state);

  state->w = initscr();
  start_color();
  cbreak();
  noecho();

  init_pair(COLOR_PLAIN, COLOR_WHITE, COLOR_BLACK);
  init_pair(COLOR_TITLE, COLOR_BLACK, COLOR_WHITE);
  init_pair(COLOR_WARN, COLOR_BLACK, COLOR_YELLOW);
  init_pair(COLOR_ERROR, COLOR_BLACK, COLOR_RED);

  g_main_loop_run (state->mainloop);

  endwin ();
  g_source_remove (state->timer_id);
  bot_glib_mainloop_detach_lcm (state->publish_lcm);
  g_main_loop_unref (state->mainloop);
  state->mainloop = NULL;
  //globals_release_lcm (s->lc);
  free (state);
}
