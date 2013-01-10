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
#include <iostream>
#include <vector>
#include <lcmtypes/visualization.h>


#include <ncurses.h>
#include <wchar.h>

using namespace std;




typedef struct  {
  WINDOW *w;
  int width, height;
  int ix, iy;

  string config_fname;
  string depth_source_name;
  string scene_mode;
  string motion_mode;

  lcm_t* publish_lcm;
  lcm_t* subscribe_lcm;

  GMainLoop * mainloop;
  guint timer_id;

  // ID of a pose at 0,0,0
  int null_obj_collection;

  // focus pose:
  double focus[3];
  int w_color;

  int w_color_edit;
  vector<int> InBoxIndices;
  vector<int> InBoxColors;

  vector < vector<float > > color_list;  


  double hand_wheel;

} state_t;



float old_colors[] = {0, 0, 255, // - ceiling ->  offwhite walls [ceiling]
0, 255, 0, // grey concrete pillars 
255, 255, 0, // elevator doors
0, 255, 255, //3 contrete floor --->
63, 63, 63, // 4 concrete staircase
0, 0, 127, // 5 .. navy carpet retained
255, 0, 0,
0, 127, 0, // 7
127, 0, 0, // 8 TROUBLE
0, 127, 127,// silver steel railings
127, 0, 127,// 10  
127, 127, 0,
255, 0, 255, // 12 'walkway' --> contrete grey
127, 127, 255, //13 walls
127, 255, 127, // 14 glass - special case
255, 127, 127,
127, 63, 0, // 16 TROUBLE down as walls but was actually window fittings in ceiling
127, 0, 63, // 17 glass ceiling
127, 51, 31, // doors
127, 63, 127}; // purple stair case ---> 110, 110, 110 grey
const int num_old_colors = sizeof(old_colors)/(3*sizeof(float));

float new_colors[] = {210, 210, 210, // done 0, 0, 0 offwhite wall [ceiling]
100, 100, 100,// 1 grey conrtet pillers 
120, 120, 120,// 2 elevators doors
105, 105, 105,// 3 concrrette floor e.g. floor01
106, 106, 106,// 4 concrrette staircase
  0,   0, 127,// 5 navy carpet retained
170,  31,  24,// 6 red burgendy carpety on 2nd floor
  0,   0,   0,// 7 glass door - merged into glass
255, 253, 200,// 8 TROUBLE cream off white walls 2
 90,  90,  90,// 9 silver steel ralings
130, 130, 130,// 10 concrete staircases (on a stair case)
101, 101, 101,// 11 concrete pillers 2
104, 104, 104,// 12 concrette floor
255, 253, 208,// 13 cream off white walls
  0,   0,   0,// 14 glass - special case
207, 112,  48,// 15 wooden window lintels 
103, 103, 103,// 16 TROUBLE - down as 'wwalls -> drak grey
  0,   0,   0,// 17 glass ceiling - merged into glass
 96,  42,  14,// 18 wooden doors 
110, 110, 110,// 19 contrete staircase e.g floor 01
 96, 112, 144, // 20 ADDED COLOR grey carpet on 3rd floor
157,  19,  15,// 21 ADDED COLOR red walls e.g. 3rd floor
};

string color_names[] = {
"wall off white [ceiling]", // 0 ** original **
"grey contrete", // 1 ** original **
"elevtor door, grey", // 2
"concrete floor grey", // 3
"concrete stair grey", // 4
"navy carpet", // 5
"red/salmon carpet", //6 ** original **
"glass door - black", // 7
"wall off white 2", // 8
"silver stell railings",//9
"grey railings", // 10 ** original **
"concrete pillers 2", // 11
"concrete floor 2", // 12
"wall off white/cream",   // 13 ** original **
"glass - to black", //14
"wooden window lintels", // 15 ** original **
"grey - avoid this", // 16 
"glass ceiling to black", //17
"wooden doors",//18
"concrete staircase",
"grey carpet", //20 ** original **
"red burgendy walls", //21 ** original **
  };





// original colors:
/*float colors[] = {
    96.0,112.0,144.0,// grey carpet on 3rd floor
    157.0, 19.0, 15.0,// red walls e.g. 3rd floor
    //170.0, 31.0, 24.0,// red/salmon carpet 
    //255, 253, 208,// cream offwhite walls
    //210.0,210.0,210.0,// offwhite walls [ceiling]
    //100.0,100.0,100.0,// grey contrete 
    //130.0,130.0,130.0,// grey railings    
    207.0,112.0, 48.0,// window lintels    
};*/
const int num_colors = sizeof(new_colors)/(3*sizeof(float));



#define COLOR_PLAIN 1
#define COLOR_TITLE 2
#define COLOR_ERROR 3
#define COLOR_WARN  4

#define STEP_TESTING 1

static int repaint (state_t * s, int64_t now);
static int publishfocus(void *user_data);



static int color_cluster(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);

  
 
  return 1;
}

static int publish_hand_wheel(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);

 return 0; 
}

static int publishfocus(void *user_data){
  state_t* s = static_cast<state_t*>(user_data);

 return 0; 
}


static int
repaint (state_t * s, int64_t now)
{
  WINDOW * w = s->w;

  clear();
  
  getmaxyx(w, s->height, s->width);
  color_set(COLOR_PLAIN, NULL);
  
  wmove(w, s->height/2,0);
  
  for (int i = 0; i < s->width/2; i++)
      wprintw(w, "-");
  
  for (int i = 0; i < s->height; i++) {
      wmove(w, i, s->width/4);
      wprintw(w, "|");
  }

  //update:
  color_set(COLOR_PLAIN, NULL);
  wmove(w, 0, 0);
  wprintw(w, "<-  ->: move ud");
  wmove(w, 1, 0);
  wprintw(w, "/\\  \\/: move lr");
  wmove(w, 2, 0);
  wprintw(w, "color: %s",color_names[s->w_color].c_str());
  wmove(w, 3, 0);
  wprintw(w, "%f wheel",s->hand_wheel);
  wmove(w, 4, 0);
  wprintw(w, "%d and %d",s->InBoxIndices.size(),s->InBoxColors.size());
  

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
    
  // trans rpy
  // 0 1 2 345
  double deltas[]={0.1,0.1,0.1,0.1,0.1,0.1};

  wmove(w, 0, 0);
  wprintw(w,"%i  ",c);

  double wheel_scale =0.1;
  double scale =1.5;
  // 65 up, 66 down,
  switch (c)
  {
    case 65: // up arrow: roll
      s->focus[0] += scale ;
      publishfocus(s);
      s->w_color_edit =0;
      break;
    case 66: // down arrow: -roll
      s->focus[0] -= scale ;
      publishfocus(s);
      s->w_color_edit =0;
      break;
    case 68: // left arrow:
      s->hand_wheel -= wheel_scale ;
      publish_hand_wheel(s);
      break;
    case 67: // right arrow:
      s->hand_wheel += wheel_scale ;
      publish_hand_wheel(s);
      break;
    case 'c': // color the cluster the same color
      color_cluster(s);
      publishfocus(s);
      s->w_color_edit =0;
      break;
    case 'x': // change the color output color
      s->w_color++;
      if(s->w_color == num_colors){
	s->w_color=0;
      }
      break;
    case 's': // change the color output color
      //savePLYFile(s->model,"output_ply.ply");
      break;
    case 'z': // change the input color 
      s->w_color_edit++;
      if(s->w_color_edit == s->color_list.size()){
	s->w_color_edit=0;
      }
      publishfocus(s);
      break;
    case 'p': // swap out the colors
      //swap_colors(s->model);
      //publishfocus(s);
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
//    send_actuator_command (s, now);
    repaint (s, now);
    return TRUE;
}

void set_default_config(void *user_data){
  state_t* state = static_cast<state_t*>(user_data);
  state->publish_lcm= lcm_create(NULL);
  state->subscribe_lcm = state->publish_lcm;

}


int main(int argc, char *argv[])
{
  state_t* state = new state_t();
  set_default_config(state);
  
  state->null_obj_collection= 0;
    
  state->w_color = 0;
  state->w_color_edit = 0;
  state->focus[0] =22;
  state->focus[1] =50;
  
  // 2. Process Arguments
  char* config_fname ="/home/mfallon/projects/kmcl/kmcl/config/kmcl_demo.cfg";
  char* depth_source_name = "kinect";
  char* scene_mode = "floors0203";
  char* motion_mode = "typical";
  char* sparse_poses;  
  char* begin_timestamp="0";
  double mean_s0[] = {100,100,0};
  GOptionEntry entries[] = {
    { "config", 'c', 0, G_OPTION_ARG_STRING, &config_fname, "Location of config file", NULL },
    { "sensor", 'x', 0, G_OPTION_ARG_STRING, &depth_source_name, "Name of the sensor being used ", NULL },
    { "building", 'b', 0, G_OPTION_ARG_STRING, &scene_mode, "Building model config [default=floors0203]", NULL },
    { "motion", 'm', 0, G_OPTION_ARG_STRING, &motion_mode, "Expected type of motion [default=typical]", NULL },
    { NULL }
  };  
  GError *error = NULL;
  GOptionContext *context = NULL;
  context = g_option_context_new ( "- convert (freenect) kinect disparity data to xyz points" );
  g_option_context_add_main_entries ( context, entries, NULL );
  if ( !g_option_context_parse ( context, &argc, &argv, &error ) ) {
    g_assert ( error );
    printf( "Error: %s\n%s",
          error->message, g_option_context_get_help ( context, TRUE, NULL ) );
    return -1;
  }
  if ( argc > 1 ) {
    printf( "Error: Unknown options\n%s",
          g_option_context_get_help ( context, TRUE, NULL ) );
    return -1;
  }  
  
  state->config_fname = config_fname;
  state->depth_source_name = depth_source_name;
  state->scene_mode = scene_mode;
  state->motion_mode = motion_mode;  


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
