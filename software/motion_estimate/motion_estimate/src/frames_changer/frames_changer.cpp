// simple ncurses tool
// to modify a bot_frame transform 
//
// was used to explore/improve the calibration of the head

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

#include <cstdio> 
#include <iostream>
#include <vector>


#include <ncurses.h>
#include <wchar.h>

using namespace std;

typedef struct  {
  WINDOW *w;
  int width, height;
  int ix, iy;

  lcm_t* publish_lcm;
  lcm_t* subscribe_lcm;

  GMainLoop * mainloop;
  guint timer_id;

  double trans[3];
  double rpy[3]; // in degrees

  int last_input;

} state_t;

#define COLOR_PLAIN 1
#define COLOR_TITLE 2
#define COLOR_ERROR 3
#define COLOR_WARN  4

static int publish_hand_wheel(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);

  bot_core_rigid_transform_t msg;
  memcpy( msg.trans, s->trans, 3*sizeof(double) );

  double rpy[] = {s->rpy[0]*M_PI/180, s->rpy[1]*M_PI/180, s->rpy[2]*M_PI/180 };
  double q[4];
  bot_roll_pitch_yaw_to_quat (rpy, q);
  memcpy( msg.quat, q, 4*sizeof(double) );

  msg.utime =0;

  bot_core_rigid_transform_t_publish(s->publish_lcm, "CAMERA_LEFT_TO_PRE_SPINDLE", &msg);
  msg.utime =bot_timestamp_now();


  bot_core_rigid_transform_t_publish(s->publish_lcm, "CAMERA_LEFT_TO_PRE_SPINDLE", &msg);

 return 0; 
}



static int
repaint (state_t * s, int64_t now)
{
  WINDOW * w = s->w;

  clear();
  
  getmaxyx(w, s->height, s->width);
  color_set(COLOR_PLAIN, NULL);
  
  //update:
  color_set(COLOR_PLAIN, NULL);
  wmove(w, 0, 0);
  wprintw(w, "<-  ->: move ud");
  wmove(w, 1, 0);
  wprintw(w, "/\\  \\/: move lr");
  wmove(w, 3, 0);
  wprintw(w, "trans: %.4f %.4f %.4f",s->trans[0] ,s->trans[1] ,s->trans[2]);
  wmove(w, 4, 0);
  wprintw(w, "  rpy: %.4f %.4f %.4f",s->rpy[0] ,s->rpy[1] ,s->rpy[2]);
  wmove(w, 6, 0);
  wprintw(w, "last %d",s->last_input);
  

  color_set(COLOR_TITLE, NULL);
  
  wrefresh (w);
  return 0;
}


static gboolean
on_input (GIOChannel * source, GIOCondition cond, gpointer data)
{
  state_t* s = static_cast<state_t*>(data);
  WINDOW * w = s->w;
  int64_t now = bot_timestamp_now ();

  int c = getch();
    
  double delta_trans[]={0.001,0.001,0.001};
  double delta_rpy[]={0.1,0.1,0.1}; // degrees

  wmove(w, 5, 0);
  wprintw(w,"%i  ",c);

  s->last_input = c;

  // 65 up, 66 down,
  switch (c)
  {
    case 65: // up arrow: roll
      s->trans[2] += delta_trans[0] ;
      publish_hand_wheel(s);
      break;
    case 66: // down arrow: -roll
      s->trans[2] -= delta_trans[0] ;
      publish_hand_wheel(s);
      break;
    case 68: // left arrow:
      s->trans[0] -= delta_trans[1] ;
      publish_hand_wheel(s);
      break;
    case 67: // right arrow:
      s->trans[0] += delta_trans[1] ;
      publish_hand_wheel(s);
      break;
    case 72: // home
      s->trans[1] -= delta_trans[2] ;
      publish_hand_wheel(s);
      break;
    case 70: // end
      s->trans[1] += delta_trans[2] ;
      publish_hand_wheel(s);
      break;
    ////////////////////////////
    case 'r': // roll
      s->rpy[2] += delta_rpy[0] ;
      publish_hand_wheel(s);
      break;
    case 'f': // roll
      s->rpy[2] -= delta_rpy[0] ;
      publish_hand_wheel(s);
      break;
    case 'p': // pitch
      s->rpy[1] += delta_rpy[1] ;
      publish_hand_wheel(s);
      break;
    case 59: // pitch
      s->rpy[1] -= delta_rpy[1] ;
      publish_hand_wheel(s);
      break;
    case 'y': // yaw
      s->rpy[0] += delta_rpy[2] ;
      publish_hand_wheel(s);
      break;
    case 'h': // roll
      s->rpy[0] -= delta_rpy[2] ;
      publish_hand_wheel(s);
      break;
  }
    
    
    repaint (s, now);	
    return TRUE;
}

static gboolean
on_timer (void * user)
{
  state_t* s = static_cast<state_t*>(user);
    int64_t now =0;// bot_timestamp_now ();
    repaint (s, now);
    return TRUE;
}



int main(int argc, char *argv[])
{
  state_t* state = new state_t();
  state->publish_lcm= lcm_create(NULL);
  state->subscribe_lcm = state->publish_lcm;
  
  double    translation[] = { 0.0350, -0.0777, -0.0033};
  double rpy[] ={ -0.3002, -1.8998, 89.6263 };
  
  
  memcpy (state->trans, translation, 3*sizeof(double) );
  memcpy (state->rpy, rpy, 3*sizeof(double) );

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
