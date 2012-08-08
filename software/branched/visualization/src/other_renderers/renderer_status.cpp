
#include <stdio.h>
#include <glib.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>

//#include <bot/bot_core.h>
//#include <bot/viewer/viewer.h>

//#include <lcmtypes/fbn_feat_ping_t.h>
//#include "lcmtypes/mrtypes.h"
//#include "lcmtypes/mrlcm_stringplusname_t.h"

//#include <common/globals.h>

#include <stdio.h>
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

//#include <visualization/renderer_status.hpp>

#include <lcmtypes/vs_system_status_t.h>
//#include "visualization/collections.hpp"
//#include "lcmtypes/vs_system_status_t.h"
//#include "lcmtypes/fbn_p2v_mvmt_command_t.h"
//#include "lcmtypes/fbn_v2p_reconstate_t.h"

using namespace std;

const char* PARAM_STATUS_0 = "SAM";
const char* PARAM_STATUS_1 = "Control";
const char* PARAM_STATUS_2 = "Recon";

const char* PARAM_IMPORTANT = "Important";

const char* PARAM_MODE = "Mode";


const bool PARAM_STATUS_0_DEFAULT = false;
const bool PARAM_STATUS_1_DEFAULT = false;
const bool PARAM_STATUS_2_DEFAULT = false;
const bool PARAM_IMPORTANT_DEFAULT = false;



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
    
//    fbn_feat_ping_t_subscription_t *sub;
// disabled for now    fbn_water_state_t_subscription_t *water_sub;
    vs_system_status_t_subscription_t *status_sub;
//    fbn_v2p_reconstate_t_subscription_t *reconstate_sub;
//    fbn_p2v_mvmt_command_t_subscription_t *cmd_heading_sub;
//    fbn_p2v_mvmt_command_t_subscription_t *cmd_speed_sub;
//    fbn_p2v_mvmt_command_t_subscription_t *cmd_depth_sub;
    
    
    // Information to be displayed:
    int horz_pingnumber;
    int horz_n_features;
    int vert_pingnumber;
    int vert_n_features;
    double cur_hsd[3];
    double cmd_hsd[3];
    double altitude, pitch, roll;
    

    int list_size; // 80
    
   // used for multidimensional lists:
    vector<string> msgchannels;
    vector<int> list_ctrs;
    vector <vector <string> > msglists;
    vector <vector <long long> > msgutimes;
    bool param_status[3];
    bool param_important;
    int visability;
    
    // used for combined list:
    int comb_list_ctr;
    vector <string> comb_msglist;
    vector <long long> comb_msgutime;
    
    // used for combined important list:
    int combimp_list_ctr;
    vector <string> combimp_msglist;
    vector <long long> combimp_msgutime;

    // for custom checkboxes
  GtkWidget* vbox;     
        
} RendererSystemStatus;

enum {
    MODE_FULL, // text visability
    MODE_FADE,  
    MODE_NONE,  
};

/*
static void 
on_features(const lcm_recv_buf_t *rbuf, 
     const char *channel, const  fbn_feat_ping_t *msg, void *user)
{
    RendererSystemStatus *self = (RendererSystemStatus*)user;
    
    if (strcmp ("FEATURES_VERT",channel) == 0) {
      self->vert_pingnumber = msg->pingnumber;
      self->vert_n_features = msg->feat_head[0].n_features;
    } else if (strcmp ("FEATURES_HORZ",channel) == 0){
	self->horz_pingnumber = msg->pingnumber;
	self->horz_n_features = msg->feat_head[0].n_features;
    }
    
    
    bot_viewer_request_redraw(self->viewer);
}*/

/*
 // not used anymore, mfallon feb2011
static void
on_water_info_data(const lcm_recv_buf_t *rbuf,
        const char *channel, const fbn_water_state_t *msg, void *user)
{
    RendererSystemStatus *self = (RendererSystemStatus*) user;
    self->depth = msg->depth;
    
    bot_viewer_request_redraw(self->viewer);
}
*/


/*
static void
on_cmd_heading(const lcm_recv_buf_t *rbuf,
        const char *channel, const fbn_p2v_mvmt_command_t *msg, void *user)
{
    RendererSystemStatus *self = (RendererSystemStatus*) user;
    self->cmd_hsd[0] = msg->cmd_value;
}

static void
on_cmd_speed(const lcm_recv_buf_t *rbuf,
        const char *channel, const fbn_p2v_mvmt_command_t *msg, void *user)
{
    RendererSystemStatus *self = (RendererSystemStatus*) user;
    self->cmd_hsd[1] = msg->cmd_value;
}

static void
on_cmd_depth(const lcm_recv_buf_t *rbuf,
        const char *channel, const fbn_p2v_mvmt_command_t *msg, void *user)
{
    RendererSystemStatus *self = (RendererSystemStatus*) user;
    self->cmd_hsd[2] = msg->cmd_value;
}

static void
on_reconstate(const lcm_recv_buf_t *rbuf,
        const char *channel, const fbn_v2p_reconstate_t *msg, void *user)
{
    RendererSystemStatus *self = (RendererSystemStatus*) user;
    self->cur_hsd[0] = msg->heading;
    self->cur_hsd[1] = msg->velocity;
    self->cur_hsd[2] = msg->depth;
    self->altitude = msg->altitude;
    self->pitch = msg->pitch;
    self->roll = msg->roll;
//    bot_viewer_request_redraw(self->viewer);
}
*/


static void
on_vs_system_status(const lcm_recv_buf_t *rbuf,
        const char *channel, const vs_system_status_t *msg, void *user)
{
    RendererSystemStatus *self = (RendererSystemStatus*) user;
    

    //printf("got nmea\n");
    // 1. Match the incoming data to a channel
    int W_list = 0;
    for (int i=0;i<self->msgchannels.size();i++){
       if (self->msgchannels[i].compare(msg->name) != 0){
	  cout << self->msgchannels[i] << " is not " << msg->name << "\n";
       }else{
	  cout << self->msgchannels[i] << " is  " << msg->name << "\n";
	  W_list = i;
	  break;
       }
    }
    /*cout << W_list << " is invalid. need to fix this bug\n"; 
    cout << "using list: " << W_list << "\n";
    std::cout << "this this value next: " << self->list_ctrs[W_list] << std::endl;
    */
   
    // 2. stick the data into the msglist
    std::string temp2;
    temp2 =  msg->value;
    char char_buf[40];
    //sprintf (char_buf, " @ %lld   str at: %d", msg->utime ,self->list_ctrs[W_list]);
    snprintf (char_buf,40, " @ %lld", msg->utime);// ,self->list_ctrs[W_list]);
    temp2.append(char_buf);
    
    self->msglists[W_list][self->list_ctrs[W_list]]=temp2;
    self->msgutimes[W_list][self->list_ctrs[W_list]] = msg->utime;

    // 3. update the circular counter:
    self->list_ctrs[W_list] ++;
    if (self->list_ctrs[W_list] >= self->list_size){
     self->list_ctrs[W_list]=0; 
    }
    //    if (self->param_status[W_list]){
    //cout << "widget for " << msg->name << "is active to stick in combined buffer\n";

    // 4.1 add combined list element:
    stringstream oss;
    oss << W_list << msg->name << ": ";
     std::string temp_c;
     temp_c =oss.str() + msg->value;

    char char_buf_c[40];
    snprintf (char_buf_c,40, " @ %lld", msg->utime);// ,self->list_ctrs[W_list]);
    temp_c.append(char_buf_c);
    
    self->comb_msglist[self->comb_list_ctr]=temp_c;
    self->comb_msgutime[self->comb_list_ctr] = msg->utime;
    
    // 4.2 update the circular counter:
    self->comb_list_ctr ++;
    if (self->comb_list_ctr >= self->list_size){
     self->comb_list_ctr=0; 
    }      
    
    if (msg->level < 3){
      self->combimp_msglist[self->combimp_list_ctr]=temp_c;
      self->combimp_msgutime[self->combimp_list_ctr] = msg->utime;
      // 4.2 update the circular counter:
      self->combimp_list_ctr ++;
      if (self->combimp_list_ctr >= self->list_size){
	self->combimp_list_ctr=0; 
      }      
    }


    //   }
    
    /*
    // Keep this for debug:
    for (vector< vector<int> >::size_type u = 0; u < self->msglists.size(); u++) {
      cout << "list no " << u << endl;
      for (vector<int>::size_type v = 0; v < self->msglists[u].size(); v++) {
        cout << self->msglists[u][v] << " ";
      }
      cout << endl;
    }
    */
    //    printf("\nend nmea\n");
}

////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Drawing Functions ------------------------- //
////////////////////////////////////////////////////////////////////////////////

static void
_draw(BotViewer *viewer, BotRenderer *r)
{
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

    void *font = GLUT_BITMAP_8_BY_13;
    int line_height = 14;

    float colors[3][3] = {
        { 1.0, 0.0, 0.0 }, // red
        { 0.7, 0.7, 0.0 }, // yellow
        { 0.0, 1.0, 0.5 }, // green
    };
    //{ 0.6, 0.6, 0.6 }, // gray        

    char line1[80], line2[80], line3[80], line4[80], line5[80], line6[80];

    sprintf(line1, "horz %d @ %d",self->horz_n_features, self->horz_pingnumber);
    sprintf(line2, "vert %d @ %d",self->vert_n_features, self->vert_pingnumber);
    sprintf(line3, "ptch %5.1f rl %5.1f",self->pitch,self->roll); // spare
    sprintf(line4, " alt %5.2f wc %5.2f",self->altitude,(self->altitude+self->cur_hsd[2]) ); 
    // water column = recon_altitude + recon_depth
    sprintf(line5, " cur %3.0f %2.2f %5.2f",self->cur_hsd[0],self->cur_hsd[1],self->cur_hsd[2] );
    sprintf(line6, " cmd %3.0f %4.0f %5.2f",self->cmd_hsd[0],self->cmd_hsd[1],self->cmd_hsd[2] );

    //double x = hind * 110 + 120;
    double x = 0 ;// hind * 150 + 120;
    double y = gl_height - 5 * line_height - 1;

    glColor4f(0, 0, 0, 0.7);
    glBegin(GL_QUADS);
    glVertex2f(x, y - line_height);
    glVertex2f(x + 19*8, y - line_height); // 19 is the number of chars in the box

    glVertex2f(x + 19*8, y + 5 * line_height); // 19 is the number of chars in the box
    glVertex2f(x, y + 6 * line_height);
    glEnd();

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
    
    // scrolling text://////////////////////////////
    double x_pos =140;
    double y_pos;
    y = line_height;
    char scroll_line[100];
    
    int W_to_show=0;
    if (!self->param_status[0]){
      W_to_show=1;
    }
    int N_active_params=0;
    for (int j=0; j < 3 ; j++){
      if (self->param_status[j]){
        //printf ("%d is active\n",j);
        N_active_params++;
        W_to_show=j;
      }
    }
    
    int max_msgs_onscreen =80;
    if (1==1){
    
      if (N_active_params==1){ // if only one class wants to be shown:
        //printf ("show widget number %d ? %d\n",W_to_show,self->param_status[W_to_show]);
        //   printf("refresh\n");
        //glColor3fv(colors[1]);
        int msgs_onscreen = 0;
	// go through the circular list
        for (int i=0;i<self->list_size;i++){ // was 80
         y_pos = line_height*((float)i +1);
         int ctr = self->list_ctrs[W_to_show]-1 - i;
         if (ctr<0){
	   ctr = ctr+self->list_size;
         }
         // stop of we have covered the screen:
         if (y_pos > (gl_height)){
	   break;
         }
         float colors4[4] = {0};
         colors4[0] =colors[W_to_show][0];
         colors4[1] =colors[W_to_show][1];
         colors4[2] =colors[W_to_show][2];
         colors4[3] =1;
         if (i>10){
	      if (self->visability==MODE_FULL){
		colors4[3] = 1;
	      }else if (self->visability==MODE_NONE){
		colors4[3] = 0;
	      }else{
		colors4[3] = 0.4;
	      }
         }
         string temp;
         temp= self->msglists[W_to_show].at(ctr);
         long long this_utime= self->msgutimes[W_to_show].at(ctr);
         int age_of_msg =(int) (bot_timestamp_now() - this_utime)/1000000;
         sprintf(scroll_line, "%5.d %s",age_of_msg,temp.c_str());
         // use this for debugging:
         //sprintf(scroll_line, "%5.d %s line number %d - ctr: %d",age_of_msg,temp.c_str(),i,ctr);
         //      glColor3fv(colors[1]);
         glColor4fv(colors4);
         glRasterPos2f(x_pos, gl_height -y_pos  );
         glutBitmapString(font, (unsigned char*) scroll_line);    
         msgs_onscreen++;
         if (msgs_onscreen > max_msgs_onscreen) {
	   break;
         }
       }
     }else if((N_active_params > 1)&&(self->param_important)){
       //          printf ("show multiple widgets\n");
       //   printf("refresh\n");
       //glColor3fv(colors[1]);
       int msgs_onscreen = 0;
       for (int i=0;i<self->list_size;i++){ // was 80
         int ctr = self->combimp_list_ctr -1 - i;
         if (ctr<0){
	   ctr = ctr+self->list_size;
         }
         // stop of we have covered the screen:
         if (y_pos > (gl_height)){
	   break;
         }
         string temp;
         temp= self->combimp_msglist.at(ctr);
         long long this_utime= self->combimp_msgutime.at(ctr);
         int age_of_msg =(int) (bot_timestamp_now() - this_utime)/1000000;
         int W_to_show_comb;
         istringstream ( temp.substr(0,1) ) >> W_to_show_comb;
         //cout << "i  should choose colour" << W_to_show_comb << endl; 
         if (self->param_status[W_to_show_comb]){
           y_pos = line_height*((float)msgs_onscreen +1);
           float colors4[4] = {0};
           colors4[0] =colors[W_to_show_comb][0];
           colors4[1] =colors[W_to_show_comb][1];
           colors4[2] =colors[W_to_show_comb][2];
           colors4[3] =1;
           if (i>10){
	      if (self->visability==MODE_FULL){
		colors4[3] = 1;
	      }else if (self->visability==MODE_NONE){
		colors4[3] = 0;
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
           if (msgs_onscreen > max_msgs_onscreen) {
	     break;
           }
         }
       }
     }else if (N_active_params > 1){ // if only two classes want to be shown:
       //          printf ("show multiple widgets\n");
       //   printf("refresh\n");
       //glColor3fv(colors[1]);
       int msgs_onscreen = 0;
       for (int i=0;i<self->list_size;i++){ // was 80
         int ctr = self->comb_list_ctr -1 - i;
         if (ctr<0){
	   ctr = ctr+self->list_size;
         }
         // stop of we have covered the screen:
         if (y_pos > (gl_height)){
	   break;
         }
         string temp;
         temp= self->comb_msglist.at(ctr);
         long long this_utime= self->comb_msgutime.at(ctr);
         int age_of_msg =(int) (bot_timestamp_now() - this_utime)/1000000;
         int W_to_show_comb;
         istringstream ( temp.substr(0,1) ) >> W_to_show_comb;
         //cout << "i  should choose colour" << W_to_show_comb << endl; 
         if (self->param_status[W_to_show_comb]){
           y_pos = line_height*((float)msgs_onscreen +1);
           float colors4[4] = {0};
           colors4[0] =colors[W_to_show_comb][0];
           colors4[1] =colors[W_to_show_comb][1];
           colors4[2] =colors[W_to_show_comb][2];
           colors4[3] =1;
           if (i>10){
	      if (self->visability==MODE_FULL){
		colors4[3] = 1;
	      }else if (self->visability==MODE_NONE){
		colors4[3] = 0;
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
           if (msgs_onscreen > max_msgs_onscreen) {
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
  self->param_important = bot_gtk_param_widget_get_bool(self->pw, PARAM_IMPORTANT);

  self->visability = bot_gtk_param_widget_get_enum (self->pw, PARAM_MODE);

  bot_viewer_request_redraw (self->viewer);
}

////////////////////////////////////////////////////////////////////////////////
// ------------------------------ Up and Down ------------------------------- //
////////////////////////////////////////////////////////////////////////////////

static void 
_destroy(BotRenderer *r)
{
    if (!r) return;

    RendererSystemStatus *self = (RendererSystemStatus*)r->user;
//    if (self->lcm) {
//        if (self->sub)
//	    fbn_feat_ping_t_unsubscribe(self->lcm, self->sub);
//        globals_release_lcm(self->lcm);
//    }
    free(self);
}

BotRenderer *renderer_status_new(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    RendererSystemStatus *self = (RendererSystemStatus*)calloc(1, sizeof(RendererSystemStatus));
    
    self->lcm = lcm;
    
    self->viewer = viewer;

    BotRenderer *r = &self->renderer;
    self->renderer.name = "System Status";
    self->renderer.draw = _draw;
    self->renderer.destroy = _destroy;

    std::string temp ="initial value";
    long long temp_utime = bot_timestamp_now();
    self->list_size=200;//should be 80
    
    vector<string> msglist;
    vector<long long> msgutime;
    for (int i=0;i<self->list_size;i++){
      msglist.push_back(temp);
      msgutime.push_back(temp_utime);
    }
    self->msglists.push_back(msglist);
    self->msglists.push_back(msglist);
    self->msglists.push_back(msglist);
    self->msgutimes.push_back(msgutime);
    self->msgutimes.push_back(msgutime);
    self->msgutimes.push_back(msgutime);
    self->list_ctrs.push_back(0);
    self->list_ctrs.push_back(0);
    self->list_ctrs.push_back(0);
    

    std::string channel_name ="SAM";
    self->msgchannels.push_back(channel_name);
    channel_name ="CONTROL";
    self->msgchannels.push_back(channel_name);
    channel_name ="RECON";
    self->msgchannels.push_back(channel_name);

	/* for john mcdonald:
    std::string channel_name ="VODOM";
    self->msgchannels.push_back(channel_name);
    channel_name ="SLAM";
    self->msgchannels.push_back(channel_name);
    channel_name ="LOOP";
    self->msgchannels.push_back(channel_name);
	*/
    
    std::string tempx ="0combined initial value";
    for (int i=0;i<self->list_size;i++){
      self->comb_msglist.push_back(tempx);
      self->comb_msgutime.push_back(temp_utime);
    }
    self->comb_list_ctr=0;
    
    tempx ="0combined initial value important";
    for (int i=0;i<self->list_size;i++){
      self->combimp_msglist.push_back(tempx);
      self->combimp_msgutime.push_back(temp_utime);
    }
    self->combimp_list_ctr=0;
    
    /*
    for (vector< vector<int> >::size_type u = 0; u < self->msglists.size(); u++) {
        for (vector<int>::size_type v = 0; v < self->msglists[u].size(); v++) {
            cout << self->msglists[u][v] << " ";
        }
        cout << endl;
    }
    */
    
//    self->lcm = globals_get_lcm();
    
    //self->sub = arlcm_host_status_t_subscribe(self->lcm,
//            "HOST_STATUS", on_host_status, self);

//    self->sub =fbn_feat_ping_t_subscribe(self->lcm,
//            "FEATURES_.*", on_features, self);    
  
// disabled for now:    
//    self->water_sub = fbn_water_state_t_subscribe(self->lcm, 
//            "WATER_INFO", on_water_info_data, self);

//    self->reconstate_sub = fbn_v2p_reconstate_t_subscribe(self->lcm, 
//           "V2P_REMUS_STATE", on_reconstate, self);    
    
/*    self->cmd_heading_sub = fbn_p2v_mvmt_command_t_subscribe(self->lcm, 
            "P2V_CMD_HEADING", on_cmd_heading, self);    
    self->cmd_speed_sub = fbn_p2v_mvmt_command_t_subscribe(self->lcm, 
            "P2V_CMD_SPEED", on_cmd_speed, self);    
    self->cmd_depth_sub = fbn_p2v_mvmt_command_t_subscribe(self->lcm, 
            "P2V_CMD_DEPTH", on_cmd_depth, self);   
*/ 
    
    self->status_sub = vs_system_status_t_subscribe(self->lcm, 
            "SYSTEM_STATUS", on_vs_system_status, self);    
    self->param_status[0] = PARAM_STATUS_0_DEFAULT;    
    self->param_status[1] = PARAM_STATUS_1_DEFAULT;    
    self->param_status[2] = PARAM_STATUS_2_DEFAULT;    
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
                                      PARAM_IMPORTANT, PARAM_IMPORTANT_DEFAULT, NULL);

    bot_gtk_param_widget_add_enum (self->pw, PARAM_MODE, (BotGtkParamWidgetUIHint)0, MODE_FADE, 
	      "Full", MODE_FULL, "Fade", MODE_FADE, 
	      "None", MODE_NONE, NULL);    
    
    
        gtk_widget_show (GTK_WIDGET (self->pw));
        
    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);
  

  
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

void status_add_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    bot_viewer_add_renderer(viewer, renderer_status_new(viewer,render_priority, lcm), 
                            render_priority);
//    bot_viewer_add_renderer(viewer, renderer_fbn_status_new(viewer, render_priority, lcm), 
//                            render_priority);
}

/*void setup_renderer_fbn_status(Viewer *viewer, int priority)
{
    Renderer *r = renderer_fbn_status_new(viewer);
    viewer_add_renderer(viewer, r, priority);
}*/
