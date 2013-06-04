#include <stdio.h>
#include <glib.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <map>
#include <list>
#include <utility>
#include <string>
#include <algorithm>
#include <vector>
#include <sstream>
#include <string>
#include <deque>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>

#include <lcmtypes/drc_lcmtypes.h>

using namespace std;
#define RENDERER_NAME "System_Status"
const char* PARAM_STATUS_0 = "Network";
const char* PARAM_STATUS_1 = "Motion Estimation";
const char* PARAM_STATUS_2 = "Tracking";
const char* PARAM_STATUS_3 = "Control";
const char* PARAM_STATUS_4 = "Grasping";
const char* PARAM_STATUS_5 = "Driving";
const char* PARAM_STATUS_6 = "Planning (base)";
const char* PARAM_STATUS_7 = "Planning (robot)";
const char* PARAM_STATUS_8 = "Fall Detector";
#define NUMBER_OF_SYSTEMS 9
#define MAXIMUM_N_OF_LINES 80

const char* PARAM_IMPORTANT = "Important";
const char* PARAM_MODE = "Mode";

const bool PARAM_STATUS_0_DEFAULT = false;
const bool PARAM_STATUS_1_DEFAULT = false;
const bool PARAM_STATUS_2_DEFAULT = false;
const bool PARAM_STATUS_3_DEFAULT = false;
const bool PARAM_STATUS_4_DEFAULT = false;
const bool PARAM_STATUS_5_DEFAULT = false;
const bool PARAM_STATUS_6_DEFAULT = false;
const bool PARAM_STATUS_7_DEFAULT = false;
const bool PARAM_STATUS_8_DEFAULT = false;
const bool PARAM_IMPORTANT_DEFAULT = false;

const char* PARAM_TOTAL_KB = "Total Up";

#define PARAM_RESET_SHAPER_STATS "Reset Network Stats"


// bit values divided by (1024*8)
int total_KB_up[5]={14.0625, 56.25, 225, 900.1220703125, 3600};
int total_KB_down[5]={7200, 14400, 28800, 57600, 115200}; 

typedef enum _rate_t {
    TOTAL_KB_0, TOTAL_KB_1, TOTAL_KB_2, TOTAL_KB_3, TOTAL_KB_4
} rate_t;

  


float colors[NUMBER_OF_SYSTEMS][3] = {
        { 1.0, 0.0, 0.0 }, // red
        { 0.7, 0.7, 0.0 }, // yellow
        { 0.0, 1.0, 0.5 }, // green
        { 0.0, 0.7, 0.7 }, // cyan
        { 0.15, 0.15, 1.0 }, // blue
        { 0.0, 0.4, 0.3 }, // dark green
        { 1.0, 0.5, 0.0 },  // orange
        { 1.0, 0.5, 0.5 },  // salmon
        { 1.0, 0.15, 0.15 }, // brighter red
};


const char* PARAM_SHADING = "Shading";
const bool PARAM_SHADING_DEFAULT = true;
const char* PARAM_SCORE_AND_RATES = "Score & Rates";
const bool PARAM_SCORE_AND_RATES_DEFAULT= true;

#define ERR(fmt, ...) do { \
    fprintf(stderr, "["__FILE__":%d Error: ", __LINE__); \
    fprintf(stderr, fmt, ##__VA_ARGS__); \
} while (0)

typedef struct
{
    BotRenderer renderer;
    BotViewer *viewer;
    BotGtkParamWidget *pw;    

    lcm_t *lcm;
    int64_t last_utime;
    
    int64_t controller_utime;
    int8_t controller_state;
    
    drc_system_status_t_subscription_t *status_sub;
    
    // Information to be displayed:
    double pitch, head_pitch;
    double roll, head_roll;
    double height, head_height;
    int naffs;
    double speed, head_speed, cmd_speed;
    
    deque<drc_system_status_t *> * sys_deque;
    vector< deque<drc_system_status_t *> * > deques;
    
    // used for multidimensional lists:
    vector<string> msgchannels;
    bool param_status[NUMBER_OF_SYSTEMS];
    bool param_important;
    bool shading;
    int visability;
    drc_driving_status_t *driving_status;
    // for custom checkboxes
    GtkWidget* vbox;     
        
    int64_t frequency_utime;
    std::vector<int> frequency_list;
    std::vector< std::string> channel_list;
    int8_t real_time_percent;
    
    float left_contact;
    float right_contact;
    
    drc_score_t *score;
    
} RendererSystemStatus;

enum {
    MODE_FULL, // text visability
    MODE_FADE,  
    MODE_NONE,  
};

static void
on_ground_driving_status(const lcm_recv_buf_t *rbuf,
        const char *channel, const drc_driving_status_t *msg, void *user){
    RendererSystemStatus *self = (RendererSystemStatus*) user;
    if(self->driving_status){
        drc_driving_status_t_destroy(self->driving_status);
    }    
    self->driving_status = drc_driving_status_t_copy(msg);
    bot_viewer_request_redraw (self->viewer);
}

static void
on_drc_system_status(const lcm_recv_buf_t *rbuf,
        const char *channel, const drc_system_status_t *msg, void *user)
{
    RendererSystemStatus *self = (RendererSystemStatus*) user;
    
    drc_system_status_t* recd = drc_system_status_t_copy(msg);
    // NB: overwrite the incoming timestamp as that might have come from a simulated clock
    recd->utime = bot_timestamp_now();
    
    self->sys_deque->push_back(recd);
    if( self->sys_deque->size() > MAXIMUM_N_OF_LINES){
      self->sys_deque->pop_front();
    }

    // If we dont know which system this belongs to, remove it
    int w = (int) msg->system;
    if (w < NUMBER_OF_SYSTEMS+2){
      self->deques[ w ]->push_back( recd );
      if( self->deques[ w ]->size() > MAXIMUM_N_OF_LINES){
        self->deques[ w ]->pop_front();
      }
    }
    
    /*stringstream ss;
    ss << "Deque Sizes: ";
    for (int i=0;i<3;i++){
      ss << self->deques[i]->size() << ", ";
    }
    cout << ss.str() <<"\n";*/
}


static void
on_pose_body(const lcm_recv_buf_t * buf, const char *channel, const bot_core_pose_t *msg, void *user_data){
    RendererSystemStatus *self = (RendererSystemStatus*) user_data;
    double rpy_in[3];
    bot_quat_to_roll_pitch_yaw (msg->orientation, rpy_in) ;
    self->roll = rpy_in[0]*180/M_PI;
    self->pitch = rpy_in[1]*180/M_PI;
    //self->yaw = rpy_in[2]*180/M_PI;

    self->height = msg->pos[2]; 
    self->speed = sqrt( msg->vel[0]*msg->vel[0] + msg->vel[1]*msg->vel[1] + msg->vel[2]*msg->vel[2] );
}

static void
on_pose_head(const lcm_recv_buf_t * buf, const char *channel, const bot_core_pose_t *msg, void *user_data){
    RendererSystemStatus *self = (RendererSystemStatus*) user_data;
    double rpy_in[3];
    bot_quat_to_roll_pitch_yaw (msg->orientation, rpy_in) ;
    self->head_roll = rpy_in[0]*180/M_PI;
    self->head_pitch = rpy_in[1]*180/M_PI;
    //self->yaw = rpy_in[2]*180/M_PI;
    
    self->head_height = msg->pos[2];
    self->head_speed = sqrt( msg->vel[0]*msg->vel[0] + msg->vel[1]*msg->vel[1] + msg->vel[2]*msg->vel[2] );
}


static void
on_affordance_collection(const lcm_recv_buf_t * buf, const char *channel, const drc_affordance_collection_t *msg, void *user_data){
    RendererSystemStatus *self = (RendererSystemStatus*) user_data;
    self->naffs = msg->naffs;
}

static void
on_robot_state(const lcm_recv_buf_t * buf, const char *channel, const drc_robot_state_t *msg, void *user_data){
    RendererSystemStatus *self = (RendererSystemStatus*) user_data;
    self->last_utime = msg->utime;
}

static void
on_controller_status(const lcm_recv_buf_t * buf, const char *channel, const drc_controller_status_t *msg, void *user_data){
  RendererSystemStatus *self = (RendererSystemStatus*) user_data;
  self->controller_state = msg->state;
  self->controller_utime = msg->utime;
}

static void
on_utime(const lcm_recv_buf_t * buf, const char *channel, const drc_utime_t *msg, void *user_data){
    RendererSystemStatus *self = (RendererSystemStatus*) user_data;
    self->last_utime = msg->utime;
}

static void
on_frequency(const lcm_recv_buf_t * buf, const char *channel, const drc_frequency_t *msg, void *user_data){
    RendererSystemStatus *self = (RendererSystemStatus*) user_data;
  self->frequency_utime =  msg->utime;
  self->frequency_list.clear();
  self->channel_list.clear(); 
  for (size_t i=0;i <msg->num; i++){
    self->frequency_list.push_back( (int16_t) msg->frequency[i] );
    self->channel_list.push_back( msg->channel[i] );
  }
    //std::cout << "freqs recevied\n";
    
  self->real_time_percent = msg->real_time_percent;
}

static void
on_foot_contact(const lcm_recv_buf_t * buf, const char *channel, const drc_foot_contact_estimate_t *msg, void *user_data){
    RendererSystemStatus *self = (RendererSystemStatus*) user_data;
    self->left_contact = msg->left_contact;
    self->right_contact = msg->right_contact;
}

static void
on_score(const lcm_recv_buf_t * buf, const char *channel, const drc_score_t *msg, void *user_data){
    RendererSystemStatus *self = (RendererSystemStatus*) user_data;
    if(self->score){
        drc_score_t_destroy(self->score);
    }
    self->score = drc_score_t_copy(msg);
}


static void format_time_str (int64_t lutime, char *line)
{
  int usecs = lutime % (int)1E3; lutime /= (int)1E3;
  int msecs = lutime % (int)1E3; lutime /= (int)1E3;
  int  secs = lutime % (int)60; lutime /= (int)60;
  int  mins = lutime % (int)60; lutime /= (int)60;
  int hours = lutime % (int)24; lutime /= (int)24;
  int  days = lutime;

//  sprintf (line, "SIM TIME: %1dd %2dh %2dm %2ds %3dms %3dus\n",
//          days, hours, mins, secs, msecs, usecs );
  //sprintf (line, "%2dm %2ds %3dms\n",mins, secs, msecs);
  
  sprintf(line, "%.4f SIM %2dm %2ds", ((double)lutime/1E6), mins, secs );
  
} 


//function msg=
// all figures in KB: (bits/1204/8)
void get_ttl(BotViewer *viewer, BotRenderer *r, int total_remaining_bytes, int elapsed_sec , 
	     double &percent_left , int &expected_ttl_min, int &expected_ttl_sec, bool down_link){
  //total_used,total_budget , current_time_sec, band_id){
    RendererSystemStatus *self = (RendererSystemStatus*)r;
  rate_t total_kb_val  = (rate_t) bot_gtk_param_widget_get_enum(self->pw, PARAM_TOTAL_KB);
  int total_kb_key =0;
  if ( total_kb_val == TOTAL_KB_0 ){ total_kb_key=0;
  }else if( total_kb_val == TOTAL_KB_1 ){ total_kb_key=1;
  }else if( total_kb_val == TOTAL_KB_2 ){ total_kb_key=2;
  }else if( total_kb_val == TOTAL_KB_3 ){ total_kb_key=3;
  }else if( total_kb_val == TOTAL_KB_4 ){ total_kb_key=4; }

    
  
  int total_budget_bytes = 115200*1024;
  if (down_link){
    total_budget_bytes = total_KB_down[total_kb_key]*1024;
  }else{
    total_budget_bytes = total_KB_up[total_kb_key]*1024;
  }
  
//  int total_budget_bytes 
  int total_used_bytes = total_budget_bytes - total_remaining_bytes;
  
  double rate = (double) total_used_bytes/elapsed_sec; // KB/sec
  double expected_duration = (double) elapsed_sec * total_budget_bytes  / total_used_bytes;
  double expected_ttl = expected_duration - elapsed_sec;
  percent_left = (double) 100.0* (1.0 - (double) total_used_bytes/ (double) total_budget_bytes );
  expected_ttl_min = floor( expected_ttl / 60 );
  expected_ttl_sec = floor( expected_ttl - expected_ttl_min*60);
  
  /*
  std::cout << "total_kb_key: " << total_kb_key << "\n";
  std::cout << "downlink? " << (int) down_link << "\n";
  std::cout << total_budget_bytes << " total_budget_bytes\n";
  std::cout << total_used_bytes << " total_used_bytes\n";
  std::cout << total_remaining_bytes << " remaining bytes\n";
  std::cout << expected_duration << " expected_duration\n";
  std::cout << elapsed_sec << " elapsed_sec\n";
  std::cout << percent_left << " percent_left\n";
  std::cout << expected_ttl << " expected_ttl\n";
  std::cout << expected_ttl_min << " expected_ttl_min\n";
  std::cout << expected_ttl_sec << " expected_ttl_sec\n";
  std::cout << "=====================================\n";
  */
  
}

////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Drawing Functions ------------------------- //
////////////////////////////////////////////////////////////////////////////////

static void _draw(BotViewer *viewer, BotRenderer *r){
    RendererSystemStatus *self = (RendererSystemStatus*)r;

    glPushAttrib (GL_ENABLE_BIT);
    glEnable (GL_BLEND);
    glDisable (GL_DEPTH_TEST);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    int gl_width = (GTK_WIDGET(viewer->gl_area)->allocation.width);
    int gl_height = (GTK_WIDGET(viewer->gl_area)->allocation.height);

    // transform into window coordinates, where <0, 0> is the top left corner
    // of the window and <viewport[2], viewport[3]> is the bottom right corner
    // of the window
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, gl_width, 0, gl_height);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(0, gl_height, 0);
    glScalef(1, -1, 1);

    //void *font = GLUT_BITMAP_8_BY_13;
    void *font = GLUT_BITMAP_9_BY_15;
    int line_height = 14;
    
    // Printf the frequency_list:
    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SCORE_AND_RATES)){
      if (self->frequency_list.size()>0){
	double x=0;
	double y=0;
	for (size_t i=0; i <self->frequency_list.size() ; i++) {
	  char line[80];
	  ///std::cout <<  self->frequency_list[i] << "\n";
	  sprintf(line, "%03d %s", self->frequency_list[i], self->channel_list[i].c_str() );
	  x = 0 ;// hind * 150 + 120;
   	  y = gl_height + (-i - 11) * line_height;
	  // top left:
	  //y = 10 + i*line_height;//gl_height - 8 * line_height;
	  glColor3f(  1.0, 0.0, 0.0 );
	  glRasterPos2f(x, y);
	  glutBitmapString(font, (unsigned char*) line);
	}
	
	char lineX[80];
	float elapsed_time =  (self->last_utime - self->frequency_utime)*1E-6;
	y = gl_height - 10 * line_height;
	sprintf(lineX, "%.1f AGE OF FREQS", elapsed_time);
	glColor3f(  1.0, 0.0, 0.0 );
	glRasterPos2f(x, y);
	glutBitmapString(font, (unsigned char*) lineX);
	
	char lineY[80];
	y = gl_height - 9 * line_height;
	sprintf(lineY, "%d%% GAZEBO RATE", self->real_time_percent);
	glColor3f(  1.0, 0.0, 0.0 );
	glRasterPos2f(x, y);
	glutBitmapString(font, (unsigned char*) lineY);
	
      }
      
      if (self->score != NULL){
	char line[80];
	sprintf(line, "%d FALLS %d TASK %d SCORE",self->score->falls, self->score->task_type, self->score->completion_score );
	double x = 0 ;// hind * 150 + 120;
	double y = gl_height + ( - self->frequency_list.size() - 11) * line_height;
	glColor3f(  0.0, 0.0, 1.0 );
	glRasterPos2f(x, y);
	glutBitmapString(font, (unsigned char*) line);

	//
	// NB: sim_time_elapsed time is NOT the number of seconds left in the 30mins
	// 	
	float elapsed_sec = (float) (self->score->sim_time/1E6) ;
	float total_sec = 30.0*60.0;
	int total_remaining_sec = floor(total_sec - elapsed_sec);
	int remaining_min = floor(total_remaining_sec/60.0);
        int remaining_sec = floor(total_remaining_sec - remaining_min*60);
	float percent_remaining = 100*total_remaining_sec / total_sec;
	
	sprintf(line, "%2.2f TIME %2d:%02d",percent_remaining, remaining_min,remaining_sec);
	y = gl_height + (-1 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);	
	
	double percent_left;
	int expected_ttl_min, expected_ttl_sec;
	if (self->score->bytes_downlink_remaining !=0){
	  get_ttl(viewer, r, self->score->bytes_downlink_remaining, elapsed_sec , percent_left , expected_ttl_min, expected_ttl_sec, true);
          sprintf(line, "%2.2f DOWN %d:%d",percent_left,expected_ttl_min, expected_ttl_sec);
	}else{
          sprintf(line, "No Downlink Info");
	}
	y = gl_height + (-2 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);	

	if (self->score->bytes_uplink_remaining !=0){
  	  get_ttl(viewer, r, self->score->bytes_uplink_remaining, elapsed_sec , percent_left , expected_ttl_min, expected_ttl_sec, false);
  	  sprintf(line, "%2.2f  UP  %d:%d",percent_left,expected_ttl_min, expected_ttl_sec);
	}else{
          sprintf(line, "No Uplink Info");
	}
	y = gl_height + (-3 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);	
	
	sprintf(line, "  %%   ====  TTL ");
	y = gl_height + (-4 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);	
	
	
	
	
	
/*	
	sprintf(line, "LEFT %.1f MINS %.1f", remaining_sec, remaining_sec/60 );
	y = gl_height + (-2 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);
	sprintf(line, "GONE %.1f", elapsed_sec );
	y = gl_height + (-1 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);
	
	sprintf(line, "DOWN TTL abcd", elapsed_sec );
	y = gl_height + (-6 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);	
	sprintf(line, "LEFT %.1f \% %.1f", remaining_sec, remaining_sec/60 );
	y = gl_height + (-5 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);
	sprintf(line, "GONE %.1f", elapsed_sec );
	y = gl_height + (-4 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);
	
	
	sprintf(line, "  UP TTL abcd", elapsed_sec );
	y = gl_height + (-9 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);	
	sprintf(line, "LEFT %.1f \% %.1f", remaining_sec, remaining_sec/60 );
	y = gl_height + (-8 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);
	sprintf(line, "GONE %.1f", elapsed_sec );
	y = gl_height + (-7 - self->frequency_list.size() - 11) * line_height;
	glRasterPos2f(x, y);	glutBitmapString(font, (unsigned char*) line);
*/	
      }
    
    }
    
    // Status Block:    
    char line1[80], line2[80], line3[80], line4[80], line5[80], line6[80], line7[90], line8[90], line9[90];
    sprintf(line1, "n affs %d",self->naffs);
    sprintf(line2, " pitch %5.1f hd %5.1f",self->pitch,self->head_pitch);
    sprintf(line3, "  roll %5.1f hd %5.1f",self->roll,self->head_roll); 
    sprintf(line4, "height %5.1f hd %5.1f",self->height,self->head_height); 
    sprintf(line5, " speed %5.1f hd %5.1f",self->speed, self->head_speed );
    sprintf(line6, "spdcmd %5.1f",self->cmd_speed );
    if ((self->left_contact==1)&& (self->right_contact==1) ){
      sprintf(line7, "  feet <--BOTH-->");
    }else if(self->left_contact==1){
      sprintf(line7, "  feet <-LEFT");
    }else if(self->right_contact==1){
      sprintf(line7, "  feet     RIGHT->");
    }else{
      sprintf(line7, "  feet  **NONE**");
    }     
    //format_time_str ( self->last_utime, line8 ); 
    sprintf(line8, "%.4f SIM", ((double)self->last_utime/1E6) );

    float elapsed_control_time =  (self->last_utime - self->controller_utime)*1E-6;
    std::string status;
    if (self->controller_state == DRC_CONTROLLER_STATUS_T_UNKNOWN){ status ="UNKNOWN"; 
    }else if (self->controller_state == DRC_CONTROLLER_STATUS_T_STANDING){ status ="STANDING";
    }else if (self->controller_state == DRC_CONTROLLER_STATUS_T_WALKING){ status ="WALKING"; 
    }else if (self->controller_state == DRC_CONTROLLER_STATUS_T_HARNESSED){ status ="HARNESS"; 
    }else if (self->controller_state == DRC_CONTROLLER_STATUS_T_QUASISTATIC){ status ="QUASISTATIC"; 
    }else if (self->controller_state == DRC_CONTROLLER_STATUS_T_BRACING){ status ="BRACING"; 
    }else{ status ="UNKNOWNX"; // shouldnt happen
    }
    sprintf(line9, "%s [AGE %.1f]", status.c_str() , elapsed_control_time);
      
     
      
    double x = 0;
    double y = gl_height - 8 * line_height;

    int x_pos = 189; // text lines x_position
    if (self->shading){
      glColor4f(0, 0, 0, 0.7);
      glBegin(GL_QUADS);
      glVertex2f(x, y - line_height);
      glVertex2f(x + 21*9, y - line_height); // 21 is the number of chars in the box
      glVertex2f(x + 21*9, y + 8 * line_height); // 21 is the number of chars in the box
      glVertex2f(x       , y + 8 * line_height);
      glEnd();

      // scrolling text background://////////////////////////////
      glColor4f(0, 0, 0, 0.7);
      glBegin(GL_QUADS);
      glVertex2f(x_pos, y - line_height);
      glVertex2f(gl_width, y - line_height); // 21 is the number of chars in the box
      glVertex2f(gl_width, y + 8 * line_height); // 21 is the number of chars in the box
      glVertex2f(x_pos, y + 8 * line_height);
      glEnd();    
    }

    glColor3fv(colors[0]);
    glRasterPos2f(x, y);
    glutBitmapString(font, (unsigned char*) line1);
    glColor3fv(colors[1]);
    glRasterPos2f(x, y + 1 * line_height);
    glutBitmapString(font, (unsigned char*) line2);
    glColor3fv(colors[2]);
    glRasterPos2f(x, y + 2 * line_height);
    glutBitmapString(font, (unsigned char*) line3);
    glColor3fv(colors[0]);
    glRasterPos2f(x, y + 3 * line_height);
    glutBitmapString(font, (unsigned char*) line4);
    glColor3fv(colors[1]);
    glRasterPos2f(x, y + 4 * line_height);
    glutBitmapString(font, (unsigned char*) line5);
    glColor3fv(colors[2]);
    glRasterPos2f(x, y + 5 * line_height);
    glutBitmapString(font, (unsigned char*) line6);
    glColor3fv(colors[0]);
    glRasterPos2f(x, y + 6 * line_height);
    glutBitmapString(font, (unsigned char*) line7);
    glColor3fv(colors[1]);
    glRasterPos2f(x, y + 7 * line_height);
    glutBitmapString(font, (unsigned char*) line8);
    glColor3fv(colors[2]);
    glRasterPos2f(x, y + 8 * line_height);
    glutBitmapString(font, (unsigned char*) line9);
    
    
    
    /// scrolling text://////////////////////////////
    int y_pos=0;
    char scroll_line[100];
    int W_to_show=0;
    if (!self->param_status[0]){
      W_to_show=1;
    }
    int N_active_params=0;
    for (int j=0; j < NUMBER_OF_SYSTEMS ; j++){
      if (self->param_status[j]){
        //printf ("%d is active\n",j);
        N_active_params++;
        W_to_show=j;
      }
    }
    
    int max_bottom_strip = 8;//15;
    if (1==1){
    
      if (N_active_params==1){ // if only one class wants to be shown:
        //printf ("show widget number %d ? %d\n",W_to_show,self->param_status[W_to_show]);
        //   printf("refresh\n");
        //glColor3fv(colors[1]);
        int msgs_onscreen = 0;
	// go through the circular list
        for (int i=self->deques[W_to_show]->size()-1 ;i>=0;i--){
         // stop of we have covered the screen:
         if (y_pos > (gl_height)){
	   break;
         }
         y_pos = (int) line_height*msgs_onscreen;
         float colors4[4] = {0};
         colors4[0] =colors[W_to_show][0];
         colors4[1] =colors[W_to_show][1];
         colors4[2] =colors[W_to_show][2];
         colors4[3] =1;
         if (msgs_onscreen >  (max_bottom_strip) ){
            if (self->visability==MODE_FULL){
              colors4[3] = 1;
            }else if (self->visability==MODE_NONE){
              colors4[3] = 0;
            }else{
              colors4[3] = 0.4;
            }
         }
         string temp;
         temp= self->deques[W_to_show]->at(i)->value;
         long long this_utime= self->deques[W_to_show]->at(i)->utime;
         int age_of_msg =(int) (bot_timestamp_now() - this_utime)/1000000;
         sprintf(scroll_line, "%5.d %s",age_of_msg,temp.c_str());
         // use this for debugging:
         //sprintf(scroll_line, "%5.d %s line number %d - ctr: %d",age_of_msg,temp.c_str(),i,ctr);
         //      glColor3fv(colors[1]);
         glColor4fv(colors4);
         glRasterPos2f(x_pos,  (int)gl_height -y_pos  );
         glutBitmapString(font, (unsigned char*) scroll_line);    
         msgs_onscreen++;
         if (msgs_onscreen > MAXIMUM_N_OF_LINES) {
	   break;
         }
       }
     }else if((N_active_params > 1)&&(self->param_important)){
       //printf("important\n");
       int msgs_onscreen = 0;
       for (int i=self->sys_deque->size()-1 ;i>=0;i--){
         if (y_pos > (gl_height)){ // stop of we have covered the screen:
	   break;
         }
         string temp;
         temp= self->sys_deque->at(i)->value;
         long long this_utime= self->sys_deque->at(i)->utime;
         int age_of_msg =(int) (bot_timestamp_now() - this_utime)/1000000;
         int W_to_show_comb = (int) self->sys_deque->at(i)->system;
         //cout << "i  should choose colour" << W_to_show_comb << endl; 
           if (self->param_status[W_to_show_comb]){
             y_pos = line_height*msgs_onscreen;
             float colors4[4] = {0};
             colors4[0] =colors[W_to_show_comb][0];
             colors4[1] =colors[W_to_show_comb][1];
             colors4[2] =colors[W_to_show_comb][2];
             colors4[3] =1.0;
             if (msgs_onscreen >  (max_bottom_strip) ){
               if (self->visability==MODE_FULL){
                 colors4[3] = 1.0;
               }else if (self->visability==MODE_NONE){
                 colors4[3] = 0.0;
               }else{
                 colors4[3] = 0.4;
               }
             }
             sprintf(scroll_line, "%5.d %s",age_of_msg,temp.c_str());
             // use this for debugging:
             //sprintf(scroll_line, "%5.d %s line number %d - ctr: %d",age_of_msg,temp.c_str(),i,ctr);
             //      glColor3fv(colors[1]);
             glColor4fv(colors4);
             glRasterPos2f(x_pos, (int) gl_height -y_pos   );
             glutBitmapString(font, (unsigned char*) scroll_line);    
             msgs_onscreen++;
             if (msgs_onscreen > MAXIMUM_N_OF_LINES) {
              break;
             }
         }
       }
     }else if (N_active_params > 1){ // if only two classes want to be shown:
       //          printf ("show multiple widgets\n");
       //   printf("refresh\n");
       //glColor3fv(colors[1]);
       int msgs_onscreen = 0;
       for (int i=self->sys_deque->size()-1 ;i>=0;i--){
         if (y_pos > (gl_height)){ // stop of we have covered the screen:
	   break;
         }
         string temp;
         temp= self->sys_deque->at(i)->value;
         long long this_utime= self->sys_deque->at(i)->utime;
         int age_of_msg =(int) (bot_timestamp_now() - this_utime)/1000000;
         int W_to_show_comb = (int) self->sys_deque->at(i)->system;
         //cout << "i  should choose colour" << W_to_show_comb << endl; 
         if (self->param_status[W_to_show_comb]){
           y_pos = line_height*((float)msgs_onscreen );
           float colors4[4] = {0.0};
           colors4[0] =colors[W_to_show_comb][0];
           colors4[1] =colors[W_to_show_comb][1];
           colors4[2] =colors[W_to_show_comb][2];
           colors4[3] =1;
           if (msgs_onscreen >  (max_bottom_strip) ){
	      if (self->visability==MODE_FULL){
		colors4[3] = 1.0;
	      }else if (self->visability==MODE_NONE){
		colors4[3] = 0.0;
	      }else{
		colors4[3] = 0.4;
	      }
           	
           }
           // string str2 = temp.substr (1,temp.size()-1); // strip the first char off the string
           sprintf(scroll_line, "%5.d %s",age_of_msg,temp.c_str());
           // use this for debugging:
           //sprintf(scroll_line, "%5.d %s line number %d - ctr: %d",age_of_msg,temp.c_str(),i,ctr);
           //      glColor3fv(colors[1]);
           glColor4fv(colors4);
           glRasterPos2f(x_pos, gl_height -y_pos  );
           glutBitmapString(font, (unsigned char*) scroll_line);    
           msgs_onscreen++;
           if (msgs_onscreen > MAXIMUM_N_OF_LINES) {
	     break;
           }
         }
       }
     }
    }
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glPopAttrib ();
    bot_viewer_request_redraw (self->viewer);
}



static void on_param_widget_changed(BotGtkParamWidget *pw, const char *param, void *user_data) {
  RendererSystemStatus *self = (RendererSystemStatus*) user_data;

  self->param_status[0] = bot_gtk_param_widget_get_bool(self->pw, PARAM_STATUS_0);
  self->param_status[1] = bot_gtk_param_widget_get_bool(self->pw, PARAM_STATUS_1);
  self->param_status[2] = bot_gtk_param_widget_get_bool(self->pw, PARAM_STATUS_2);
  self->param_status[3] = bot_gtk_param_widget_get_bool(self->pw, PARAM_STATUS_3);
  self->param_status[4] = bot_gtk_param_widget_get_bool(self->pw, PARAM_STATUS_4);
  self->param_status[5] = bot_gtk_param_widget_get_bool(self->pw, PARAM_STATUS_5);
  self->param_status[6] = bot_gtk_param_widget_get_bool(self->pw, PARAM_STATUS_6);
  self->param_status[7] = bot_gtk_param_widget_get_bool(self->pw, PARAM_STATUS_7);
  self->param_status[8] = bot_gtk_param_widget_get_bool(self->pw, PARAM_STATUS_8);
  //fprintf(stderr, "Param Status : %d\n", self->param_status[5]);
  self->param_important = bot_gtk_param_widget_get_bool(self->pw, PARAM_IMPORTANT);
  self->shading = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHADING);
  self->visability = bot_gtk_param_widget_get_enum (self->pw, PARAM_MODE);
  
  if (! strcmp(param, PARAM_RESET_SHAPER_STATS)) {
    drc_utime_t msg;
    msg.utime = self->last_utime;
    drc_utime_t_publish(self->lcm, "RESET_SHAPER_STATS", &msg);
    
  }
  
  bot_viewer_request_redraw (self->viewer);
}

// ------------------------------ Up and Down ------------------------------- //
static void on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data) {
    RendererSystemStatus *self = (RendererSystemStatus*)user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data){
    RendererSystemStatus *self = (RendererSystemStatus*)user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void _destroy(BotRenderer *r){
    if (!r) return;

    RendererSystemStatus *self = (RendererSystemStatus*)r->user;
    free(self);
}

BotRenderer *renderer_status_new(BotViewer *viewer, int render_priority, lcm_t *lcm){
    RendererSystemStatus *self = (RendererSystemStatus*)calloc(1, sizeof(RendererSystemStatus));
    
    self->lcm = lcm;
    self->viewer = viewer;
    BotRenderer *r = &self->renderer;
    self->renderer.name = "System Status";
    self->renderer.draw = _draw;
    self->renderer.destroy = _destroy;

    self->sys_deque = new deque<drc_system_status_t *> ();
//    deque<drc_system_status_t *> * adeque; = new deque<drc_system_status_t *> ();
    for (int i=0;i< NUMBER_OF_SYSTEMS;i++){
      self->deques.push_back( new deque<drc_system_status_t *> () );
    }
    
    self->naffs=0;
    self->pitch = self->head_pitch = self->roll = self->head_roll = 0;
    self->height = self->head_height = self->speed = self->cmd_speed = 0;

    self->left_contact = 0.0;
    self->right_contact = 0.0;
    
    self->controller_state= DRC_CONTROLLER_STATUS_T_UNKNOWN;
    self->controller_utime= 0;
    
    
    std::string channel_name ="SAM";
    self->msgchannels.push_back(channel_name);
    channel_name ="CONTROL";
    self->msgchannels.push_back(channel_name);
    channel_name ="RECON";
    self->msgchannels.push_back(channel_name);

    self->status_sub = drc_system_status_t_subscribe(self->lcm, 
            "SYSTEM_STATUS", on_drc_system_status, self);    
    bot_core_pose_t_subscribe(self->lcm,"POSE_BODY",on_pose_body,self);
    bot_core_pose_t_subscribe(self->lcm,"POSE_HEAD",on_pose_head,self);
    drc_affordance_collection_t_subscribe(self->lcm,"AFFORDANCE_COLLECTION",on_affordance_collection,self);
    drc_robot_state_t_subscribe(self->lcm,"EST_ROBOT_STATE",on_robot_state,self);
    drc_utime_t_subscribe(self->lcm,"ROBOT_UTIME",on_utime,self);
    drc_frequency_t_subscribe(self->lcm,"FREQUENCY_LCM",on_frequency,self);
    drc_foot_contact_estimate_t_subscribe(self->lcm,"FOOT_CONTACT_ESTIMATE",on_foot_contact,self);
    drc_score_t_subscribe(self->lcm,"VRC_SCORE",on_score,self);
    drc_controller_status_t_subscribe(self->lcm,"CONTROLLER_STATUS",on_controller_status,self);

    drc_driving_status_t_subscribe(self->lcm, "DRC_DRIVING_GROUND_TRUTH_STATUS", on_ground_driving_status, self);

    self->driving_status = NULL; 
    self->score = NULL;
    
    self->param_status[0] = PARAM_STATUS_0_DEFAULT;    
    self->param_status[1] = PARAM_STATUS_1_DEFAULT;    
    self->param_status[2] = PARAM_STATUS_2_DEFAULT;    
    self->param_status[3] = PARAM_STATUS_3_DEFAULT;    
    self->param_status[4] = PARAM_STATUS_4_DEFAULT;    
    self->param_status[5] = PARAM_STATUS_5_DEFAULT;    
    self->param_status[6] = PARAM_STATUS_6_DEFAULT;    
    self->param_important = PARAM_IMPORTANT_DEFAULT;
    
  if (viewer) {
    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);

    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    GtkWidget *vbox = gtk_vbox_new (FALSE, 0);
    self->vbox = vbox;
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), vbox);
    gtk_widget_show (vbox);

    gtk_box_pack_start (GTK_BOX (vbox), GTK_WIDGET (self->pw), 
                        FALSE, TRUE, 0);  
    
   //   gtk_box_pack_start (GTK_BOX (self->renderer.widget), GTK_WIDGET (self->pw), 
   //                         FALSE, TRUE, 0);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_STATUS_0, PARAM_STATUS_0_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_STATUS_1, PARAM_STATUS_1_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_STATUS_2, PARAM_STATUS_2_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_STATUS_3, PARAM_STATUS_3_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_STATUS_4, PARAM_STATUS_4_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_STATUS_5, PARAM_STATUS_5_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_STATUS_6, PARAM_STATUS_6_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_STATUS_7, PARAM_STATUS_7_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_STATUS_8, PARAM_STATUS_8_DEFAULT, NULL);
    

    bot_gtk_param_widget_add_enum (self->pw, PARAM_MODE, (BotGtkParamWidgetUIHint)0, MODE_FADE, 
	      "Full", MODE_FULL, "Fade", MODE_FADE, 
	      "None", MODE_NONE, NULL);    
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_IMPORTANT, PARAM_IMPORTANT_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_SHADING, PARAM_SHADING_DEFAULT, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, (BotGtkParamWidgetUIHint)0,
                                      PARAM_SCORE_AND_RATES, PARAM_SCORE_AND_RATES_DEFAULT, NULL);
    bot_gtk_param_widget_add_enum (self->pw, PARAM_TOTAL_KB, (BotGtkParamWidgetUIHint)0, TOTAL_KB_0, 
	      "0 | 14KB/sec", TOTAL_KB_0 , "1 | 56B/sec", TOTAL_KB_1, 
	      "2 | 225KB/sec", TOTAL_KB_2, "3 | 900KB/sec", TOTAL_KB_3, 	      
	      "4 | 3600KB/sec", TOTAL_KB_4, NULL);    
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_RESET_SHAPER_STATS, NULL);
    
    
    gtk_widget_show (GTK_WIDGET (self->pw));
        
    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);
  
    // save widget modes:
    g_signal_connect (G_OBJECT (viewer), "load-preferences",
      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
      G_CALLBACK (on_save_preferences), self);

    /*    // mfallon, save widget modes:
    g_signal_connect (G_OBJECT (viewer), "load-preferences",
                G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                G_CALLBACK (on_save_preferences), self);   
    */  
  }

  //    bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
  //    bot_viewer_add_event_handler(viewer, ehandler, render_priority);
  return r;
}

void status_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm){
  bot_viewer_add_renderer(viewer, renderer_status_new(viewer,
    render_priority, lcm), render_priority);
}
