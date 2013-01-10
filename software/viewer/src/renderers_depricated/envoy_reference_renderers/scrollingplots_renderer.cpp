/*
 * Renders a set of scrolling plots in the top right corner of the window
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>


#include <GL/gl.h>
#include <GL/glu.h>
//#include <gdk/gdkkeysyms.h>
//#include <gtk-2.0/gdk/gdkkeysyms.h>

// bot_core1: #include <bot/bot_core.h>
#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_core/bot_core.h>

/*
#include <bot/gtk/gtk_util.h>
#include <bot/viewer/viewer.h>
#include <bot/gl/gl_util.h>
#include <bot/gl/scrollplot2d.h>
*/

/*
#include <common/globals.h>
#include <common/math_util.h>
#include <common/mrconf.h>
*/

//#include <lcmtypes/arlcm_actuation_status_vehicle_t.h>
//#include <lcmtypes/arlcm_wheelspeeds_t.h>

/*
#include "lcmtypes/mrtypes.h"

#include "lcmtypes/fbn_p2v_mvmt_command_t.h"
#include "lcmtypes/fbn_v2p_reconstate_t.h"
*/

#define PARAM_NAME_GRAPH_TIMESPAN "Time span"
#define PARAM_NAME_FREEZE "Freeze"
#define PARAM_NAME_SIZE "Size"
#define PARAM_NAME_RENDER_PITCHROLL "P&R"
#define PARAM_NAME_RENDER_HEIGHT "Height"
#define PARAM_NAME_RENDER_HEADING "Heading"
#define PARAM_NAME_RENDER_DEPTH "Depth"
#define PARAM_NAME_RENDER_SPEED "Speed"
#define PARAM_NAME_SHOW_LEGEND "Show Legends"

#define REDRAW_THRESHOLD_UTIME 5000000

// max number of points on the scrolling bar:
#define MAX_POINTS 1000

#define RENDERER_NAME "Scrolling_Plots"

typedef struct _RendererScrollingPlots RendererScrollingPlots;

struct _RendererScrollingPlots {
    BotRenderer renderer;
    BotViewer *viewer;
    lcm_t *lcm;
 //   CTrans *ctrans;

    BotGtkParamWidget    *pw;

    BotGlScrollPlot2d *pitchroll_plot;
    BotGlScrollPlot2d *height_plot;
    BotGlScrollPlot2d *heading_plot;    
    BotGlScrollPlot2d *depth_plot;
    BotGlScrollPlot2d *speed_plot;

    // Hack to make postprocessing plots work:
    int64_t      cmd_utime; 
    // used to try to figure out if we are playing back a log
    // or if we are postprocessing. If postprocessing the cmd_utime and the reconstate.utime will be very different.
    

    uint64_t      max_utime;
};


static void on_pose (const lcm_recv_buf_t * buf, const char *channel, 
                               const bot_core_pose_t *msg, void *user_data);

//static void on_v2p_remus_state (const lcm_recv_buf_t * buf, const char *channel, 
//                               const fbn_v2p_reconstate_t *msg, void *user_data);
//static void on_cmd_depth(const lcm_recv_buf_t *rbuf, const char *channel, 
//			 const fbn_p2v_mvmt_command_t *msg, void *user);
//static void on_cmd_speed(const lcm_recv_buf_t *rbuf, const char *channel, 
//			 const fbn_p2v_mvmt_command_t *msg, void *user);
//static void on_cmd_heading(const lcm_recv_buf_t *rbuf, const char *channel, 
//			 const fbn_p2v_mvmt_command_t *msg, void *user);
//static void on_ping_feat(const lcm_recv_buf_t *rbuf, const char *channel, 
//			 const fbn_feat_ping_t *msg, void *user);
			 
			 
static void update_xaxis (RendererScrollingPlots *self, uint64_t utime);
//static gboolean get_speed_update (void *user_data);

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, 
        RendererScrollingPlots *self);

static void scrolling_plots_draw (BotViewer *viewer, BotRenderer *renderer)
{
  
  
  
    RendererScrollingPlots *self = (RendererScrollingPlots*) renderer->user;
    if (!self->max_utime) return;

    GLdouble model_matrix[16];
    GLdouble proj_matrix[16];
    GLint viewport[4];

    glGetDoublev (GL_MODELVIEW_MATRIX, model_matrix);
    glGetDoublev (GL_PROJECTION_MATRIX, proj_matrix);
    glGetIntegerv (GL_VIEWPORT, viewport);

    double gs_ts_max = self->max_utime * 1e-6;
    double gs_ts_min = gs_ts_max - 
        bot_gtk_param_widget_get_double (self->pw, PARAM_NAME_GRAPH_TIMESPAN);

    bot_gl_scrollplot2d_set_xlim (self->pitchroll_plot, gs_ts_min, gs_ts_max);
    bot_gl_scrollplot2d_set_xlim (self->height_plot, gs_ts_min, gs_ts_max);
    bot_gl_scrollplot2d_set_xlim (self->heading_plot, gs_ts_min, gs_ts_max);    
    bot_gl_scrollplot2d_set_xlim (self->depth_plot, gs_ts_min, gs_ts_max);
    bot_gl_scrollplot2d_set_xlim (self->speed_plot, gs_ts_min, gs_ts_max);

    int plot_width = bot_gtk_param_widget_get_int (self->pw, PARAM_NAME_SIZE);
    int plot_height = plot_width / 3;

    int x = viewport[2] - plot_width;
    int y = viewport[1];

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_PITCHROLL)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->pitchroll_plot, 
                x, y, plot_width, plot_height);
        y += plot_height;
    }

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_HEIGHT)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->height_plot, 
                x, y, plot_width, plot_height);
        y += plot_height;
    }

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_HEADING)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->heading_plot, 
                x, y, plot_width, plot_height);
        y += plot_height;
    }

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_DEPTH)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->depth_plot, 
                x, y, plot_width, plot_height);
        y += plot_height;
    }

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_SPEED)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->speed_plot, 
                x, y, plot_width, plot_height);
        y += plot_height;
    }
}

static void
scrolling_plots_free (BotRenderer *renderer) 
{
    RendererScrollingPlots *self = (RendererScrollingPlots*) renderer;
//    globals_release_ctrans (self->ctrans);
//    globals_release_lcm (self->lcm);
    free (renderer);
}



static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
  
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}


//BotRenderer *renderer_scrolling_plots_new (BotViewer *viewer)
 //, BotFrames * frames, const char * kinect_frame)
void scrollingplots_add_renderer_to_viewer(BotViewer* viewer, int priority, lcm_t* lcm)
{
    RendererScrollingPlots *self = 
        (RendererScrollingPlots*) calloc (1, sizeof (RendererScrollingPlots));
    self->viewer = viewer;
    self->renderer.draw = scrolling_plots_draw;
    self->renderer.destroy = scrolling_plots_free;
    self->renderer.name = "Scrolling Plots";
    self->renderer.user = self;
    self->renderer.enabled = 1;

    self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
//    self->ctrans = globals_get_ctrans();

//    self->lcm = globals_get_lcm ();
    self->lcm = lcm;

    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    gtk_container_add (GTK_CONTAINER (self->renderer.widget), 
            GTK_WIDGET(self->pw));
    gtk_widget_show (GTK_WIDGET (self->pw));

    bot_gtk_param_widget_add_int (self->pw, PARAM_NAME_SIZE,
            BOT_GTK_PARAM_WIDGET_SLIDER, 50, 800, 10, 150);
    bot_gtk_param_widget_add_double (self->pw, PARAM_NAME_GRAPH_TIMESPAN, 
            BOT_GTK_PARAM_WIDGET_SLIDER, 1, 180, 0.5, 5); // was 12 seconds
    bot_gtk_param_widget_add_booleans (self->pw, 
            BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_NAME_FREEZE, 0, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint)0,
                                       PARAM_NAME_RENDER_PITCHROLL, 1, 
                                       PARAM_NAME_RENDER_HEIGHT, 1, 
                                       PARAM_NAME_RENDER_HEADING, 1,				       
                                       PARAM_NAME_RENDER_DEPTH, 1, 
                                       PARAM_NAME_RENDER_SPEED, 1, NULL);
    bot_gtk_param_widget_add_booleans (self->pw, (BotGtkParamWidgetUIHint) 0, 
            PARAM_NAME_SHOW_LEGEND, 0, NULL);

    g_signal_connect (G_OBJECT (self->pw), "changed", 
            G_CALLBACK (on_param_widget_changed), self);

    
	// mfallon, save widget modes:
        g_signal_connect (G_OBJECT (viewer), "load-preferences",
                G_CALLBACK (on_load_preferences), self);
        g_signal_connect (G_OBJECT (viewer), "save-preferences",
                G_CALLBACK (on_save_preferences), self);	    
    
    // iniitialize cmd_utime - used to play back logs properly
    self->cmd_utime = -1;
    
    // pitchroll plot
    self->pitchroll_plot = bot_gl_scrollplot2d_new ();
    bot_gl_scrollplot2d_set_title        (self->pitchroll_plot, "Pitch [b] & Roll [m]");
    bot_gl_scrollplot2d_set_text_color   (self->pitchroll_plot, 0.7, 0.7, 0.7, 1);
    bot_gl_scrollplot2d_set_bgcolor      (self->pitchroll_plot, 0.1, 0.1, 0.1, 0.7);
    bot_gl_scrollplot2d_set_border_color (self->pitchroll_plot, 1, 1, 1, 0.7);
    bot_gl_scrollplot2d_set_ylim    (self->pitchroll_plot, -10, 10);
    bot_gl_scrollplot2d_add_plot    (self->pitchroll_plot, "pitch", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->pitchroll_plot, "pitch", 0, 0, 1, 1);
    bot_gl_scrollplot2d_add_plot    (self->pitchroll_plot, "roll", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->pitchroll_plot, "roll", 0.7, 0, 0.7, 1);
    bot_gl_scrollplot2d_add_plot    (self->pitchroll_plot, "5", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->pitchroll_plot, "5", 0.8, 0.8, 0.8, 0.5);
    bot_gl_scrollplot2d_add_plot    (self->pitchroll_plot, "0", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->pitchroll_plot, "0", 0.8, 0.8, 0.8, 0.5);
    bot_gl_scrollplot2d_add_plot    (self->pitchroll_plot, "-5", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->pitchroll_plot, "-5", 0.6, 0.6, 0.6, 0.5);

    self->height_plot = bot_gl_scrollplot2d_new ();
    bot_gl_scrollplot2d_set_title        (self->height_plot, "Height");
    bot_gl_scrollplot2d_set_text_color   (self->height_plot, 0.7, 0.7, 0.7, 1);
    bot_gl_scrollplot2d_set_bgcolor      (self->height_plot, 0.1, 0.1, 0.1, 0.7);
    bot_gl_scrollplot2d_set_border_color (self->height_plot, 1, 1, 1, 0.7);
    bot_gl_scrollplot2d_set_ylim    (self->height_plot, 0, 2);
    bot_gl_scrollplot2d_add_plot    (self->height_plot, "coarse", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->height_plot, "coarse", 0, 0, 1, 1);
    bot_gl_scrollplot2d_add_plot    (self->height_plot, "fit", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->height_plot, "fit", 0.7, 0, 0.7, 1);
    bot_gl_scrollplot2d_add_plot    (self->height_plot, "1.5m", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->height_plot, "1.5m", 0.8, 0.8, 0.8, 0.5);
    bot_gl_scrollplot2d_add_plot    (self->height_plot, "1m", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->height_plot, "1m", 0.8, 0.8, 0.8, 0.5);
    //bot_gl_scrollplot2d_add_plot    (self->brake_plot, "2500", 1000);
    //bot_gl_scrollplot2d_set_color   (self->brake_plot, "2500", 0.8, 0.8, 0.8, 0.5);

    self->heading_plot = bot_gl_scrollplot2d_new ();
    bot_gl_scrollplot2d_set_title        (self->heading_plot, "Heading [deg]");
    bot_gl_scrollplot2d_set_text_color   (self->heading_plot, 0.7, 0.7, 0.7, 1);
    bot_gl_scrollplot2d_set_bgcolor      (self->heading_plot, 0.1, 0.1, 0.1, 0.7);
    bot_gl_scrollplot2d_set_border_color (self->heading_plot, 1, 1, 1, 0.7);
    bot_gl_scrollplot2d_set_ylim    (self->heading_plot, 0, 360);
    bot_gl_scrollplot2d_add_plot    (self->heading_plot, "control", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->heading_plot, "control", 0, 0, 1, 1);
    bot_gl_scrollplot2d_add_plot    (self->heading_plot, "cmded", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->heading_plot, "cmded", 1, 1, 0, 1);
    bot_gl_scrollplot2d_add_plot    (self->heading_plot, "actual", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->heading_plot, "actual", 0.7, 0, 0.7, 1);
//    bot_gl_scrollplot2d_add_plot    (self->heading_plot, "N", MAX_POINTS);
//    bot_gl_scrollplot2d_set_color   (self->heading_plot, "N", 0.8, 0.8, 0.8, 0.5);
//    bot_gl_scrollplot2d_add_plot    (self->heading_plot, "E", MAX_POINTS);
//    bot_gl_scrollplot2d_set_color   (self->heading_plot, "E", 0.8, 0.8, 0.8, 0.5);
//    bot_gl_scrollplot2d_add_plot    (self->heading_plot, "S", MAX_POINTS);
//    bot_gl_scrollplot2d_set_color   (self->heading_plot, "S", 0.8, 0.8, 0.8, 0.5);
//    bot_gl_scrollplot2d_add_plot    (self->heading_plot, "W", MAX_POINTS);
//    bot_gl_scrollplot2d_set_color   (self->heading_plot, "W", 0.8, 0.8, 0.8, 0.5);

    //bot_gl_scrollplot2d_add_plot    (self->heading_plot, "2500", 1000);
    //bot_gl_scrollplot2d_set_color   (self->heading_plot, "2500", 0.8, 0.8, 0.8, 0.5);

    // depth plot
    self->depth_plot = bot_gl_scrollplot2d_new ();
//    bot_gl_scrollplot2d_set_title        (self->depth_plot, "Steer (norm)");
    bot_gl_scrollplot2d_set_title        (self->depth_plot, "Depth [negated]");
    bot_gl_scrollplot2d_set_text_color   (self->depth_plot, 0.7, 0.7, 0.7, 1);
    bot_gl_scrollplot2d_set_border_color (self->depth_plot, 1, 1, 1, 0.7);
    bot_gl_scrollplot2d_set_bgcolor (self->depth_plot, 0.1, 0.1, 0.1, 0.7);
    bot_gl_scrollplot2d_set_ylim    (self->depth_plot, -15, 0.5); // was -1 to 1
    bot_gl_scrollplot2d_add_plot    (self->depth_plot, "control", MAX_POINTS);//0.1);
    bot_gl_scrollplot2d_set_color   (self->depth_plot, "control", 0, 0, 1, 1);
    bot_gl_scrollplot2d_add_plot    (self->depth_plot, "cmded", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->depth_plot, "cmded", 1, 1, 0, 1);
    bot_gl_scrollplot2d_add_plot    (self->depth_plot, "actual", MAX_POINTS); // was 0.1
    bot_gl_scrollplot2d_set_color   (self->depth_plot, "actual", 0.7, 0, 0.7, 1);
    bot_gl_scrollplot2d_add_plot    (self->depth_plot, "water_column", MAX_POINTS); // was 0.1
    bot_gl_scrollplot2d_set_color   (self->depth_plot, "water_column", 0.8, 0.8, 0.8, 0.5);
    //bot_gl_scrollplot2d_add_plot    (self->depth_plot, "2500", 1000);
    //bot_gl_scrollplot2d_set_color   (self->depth_plot, "2500", 0.8, 0.8, 0.8, 0.5);

    // speed plot
    self->speed_plot = bot_gl_scrollplot2d_new ();
    bot_gl_scrollplot2d_set_title        (self->speed_plot, "Speed [m/s or krpm]");
    bot_gl_scrollplot2d_set_text_color   (self->speed_plot, 0.7, 0.7, 0.7, 1);
    bot_gl_scrollplot2d_set_border_color (self->speed_plot, 1, 1, 1, 0.7);
    bot_gl_scrollplot2d_set_bgcolor (self->speed_plot, 0.1, 0.1, 0.1, 0.7);
    bot_gl_scrollplot2d_set_ylim    (self->speed_plot, 0, 1.8);

    bot_gl_scrollplot2d_add_plot    (self->speed_plot, "control", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->speed_plot, "control", 0.0, 0, 1.0, 1.0);
    bot_gl_scrollplot2d_add_plot    (self->speed_plot, "cmded", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->speed_plot, "cmded", 1, 1, 0, 1);
    bot_gl_scrollplot2d_add_plot    (self->speed_plot, "actual", MAX_POINTS);
    bot_gl_scrollplot2d_set_color   (self->speed_plot, "actual", 0.7, 0, 0.7, 1);

    // Example static lines on the plot:
//    bot_gl_scrollplot2d_add_plot    (self->speed_plot, "10", MAX_POINTS);
//    bot_gl_scrollplot2d_set_color   (self->speed_plot, "10", 0.8, 0.8, 0.8, 0.5);
//    bot_gl_scrollplot2d_add_plot    (self->speed_plot, "5", MAX_POINTS);
//    bot_gl_scrollplot2d_set_color   (self->speed_plot, "5", 0.6, 0.6, 0.6, 0.5);
//   bot_gl_scrollplot2d_add_plot    (self->speed_plot, "0", MAX_POINTS);
//   bot_gl_scrollplot2d_set_color   (self->speed_plot, "0", 0.8, 0.8, 0.8, 0.5);

    // legends?
    BotGlScrollPlot2dLegendLocation legloc = BOT_GL_SCROLLPLOT2D_HIDDEN;
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_SHOW_LEGEND)) {
        legloc = BOT_GL_SCROLLPLOT2D_TOP_RIGHT;
    }
    bot_gl_scrollplot2d_set_show_legend (self->speed_plot, legloc);
    bot_gl_scrollplot2d_set_show_legend (self->pitchroll_plot, legloc);
    bot_gl_scrollplot2d_set_show_legend (self->depth_plot, legloc);


    // subscribe to LC messages
    //self->can_decode = can_decode_new (self->lcm);
    
    bot_core_pose_t_subscribe(self->lcm,"POSE_FINDGROUND",on_pose,self);

/*    fbn_v2p_reconstate_t_subscribe (self->lcm, "V2P_REMUS_STATE",on_v2p_remus_state, self);
    fbn_p2v_mvmt_command_t_subscribe (self->lcm, "P2V_CMD_DEPTH",on_cmd_depth, self);    
    fbn_p2v_mvmt_command_t_subscribe (self->lcm, "P2V_CMD_SPEED",on_cmd_speed, self);    
    fbn_p2v_mvmt_command_t_subscribe (self->lcm, "P2V_CMD_HEADING",on_cmd_heading, self);    
    fbn_feat_ping_t_subscribe(self->lcm, "FEATURES_.*",on_ping_feat, self);
    */

    // periodically pull pose data from CTrans
    //g_timeout_add (30, get_speed_update, self);
    
    printf("Finished Setting Up Scrolling Plots\n"); 
    
    bot_viewer_add_renderer(viewer, &self->renderer, priority);

    //return &self->renderer;
}



/*
void setup_renderer_scrolling_plots (BotViewer *viewer, int render_priority)
{
    viewer_add_renderer(viewer, 
            renderer_scrolling_plots_new(viewer), render_priority);
}*/






static void
on_pose(const lcm_recv_buf_t * buf, const char *channel, const bot_core_pose_t *msg, 
                   void *user_data){
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
    update_xaxis(self,msg->utime);
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;
    
    double rpy_in[3];
    bot_quat_to_roll_pitch_yaw (msg->orientation, rpy_in) ;
    bot_gl_scrollplot2d_add_point (self->pitchroll_plot, "roll", 
                                msg->utime * 1.0e-6,
                                rpy_in[0]*180/M_PI);
    bot_gl_scrollplot2d_add_point (self->pitchroll_plot, "pitch", 
                                msg->utime * 1.0e-6,
                                rpy_in[1]*180/M_PI);

    bot_gl_scrollplot2d_add_point (self->height_plot, "coarse", 
                                msg->utime * 1.0e-6,
                                msg->pos[2]);
    
    //self->cmd_utime = msg->utime; // take this to figure out if we are postprocessing  
}


/*
static void
on_v2p_remus_state (const lcm_recv_buf_t * buf, const char *channel, const fbn_v2p_reconstate_t *msg, 
                   void *user_data)
{
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
    update_xaxis(self,msg->utime);
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;

    int64_t modified_utime = msg->utime;
    double offset_time = ((double) (self->cmd_utime - msg->utime))*1.0e-6;
    //printf("offset: %f\n",offset_time); 
    if (self->cmd_utime == -1){
      // printf("not commands heard - do nothing - playback of only v2p_\n");
    } else if (offset_time> 40 ){
      modified_utime = self->cmd_utime;
      // printf("postprocessing this log. change utime: %f\n",offset_time); 
    } else {
      // printf("do nothing - clean playback: %f\n",offset_time); 
    }
    
    bot_gl_scrollplot2d_add_point (self->pitchroll_plot, "pitch", modified_utime * 1.0e-6, 
                              msg->pitch);
    bot_gl_scrollplot2d_add_point (self->pitchroll_plot, "roll", modified_utime * 1.0e-6, 
                              msg->roll);

    bot_gl_scrollplot2d_add_point (self->depth_plot, "actual", 
                                modified_utime * 1.0e-6,
                                -msg->depth);   
    bot_gl_scrollplot2d_add_point (self->depth_plot, "water_column", 
                               modified_utime * 1.0e-6,
                                -(msg->depth + msg->altitude));   
  
    bot_gl_scrollplot2d_add_point (self->speed_plot, "actual", 
           modified_utime* 1e-6, msg->velocity);
  
    bot_gl_scrollplot2d_add_point (self->heading_plot, "actual", 
          modified_utime * 1e-6, msg->heading);
}

static void
on_cmd_depth(const lcm_recv_buf_t *rbuf,
        const char *channel, const fbn_p2v_mvmt_command_t *msg, void *user_data)
{
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
    update_xaxis(self,msg->utime);
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;
    bot_gl_scrollplot2d_add_point (self->depth_plot, "cmded", 
                                msg->utime * 1.0e-6,
                                -msg->cmd_value);   // NB: depth negated
    self->cmd_utime = msg->utime; // take this to figure out if we are postprocessing
}

static void
on_cmd_speed(const lcm_recv_buf_t *rbuf,
        const char *channel, const fbn_p2v_mvmt_command_t *msg, void *user_data)
{
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
    update_xaxis(self,msg->utime);
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;
    bot_gl_scrollplot2d_add_point (self->speed_plot, "cmded", 
                                msg->utime * 1.0e-6,
                                msg->cmd_value/1000); 
    // cmded speed is in rpm... scale it so 0->1500 its 0->1.5
}

static void
on_cmd_heading(const lcm_recv_buf_t *rbuf,
        const char *channel, const fbn_p2v_mvmt_command_t *msg, void *user_data)
{
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
    update_xaxis(self,msg->utime);
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;
    bot_gl_scrollplot2d_add_point (self->heading_plot, "cmded", 
                                msg->utime * 1.0e-6,
                                msg->cmd_value); // 0-->360
}

static void
on_ping_feat(const lcm_recv_buf_t *rbuf,
        const char *channel, const fbn_feat_ping_t *msg, void *user_data)
{
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
    update_xaxis(self,msg->utime);
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;
    char chan_short[20];
    if (strcmp ("FEATURES_VERT",channel) == 0) {
      sprintf(chan_short,"vert");
    }else{
      sprintf(chan_short,"horz");
    }
    bot_gl_scrollplot2d_add_point (self->sonar_plot, chan_short, 
                                msg->utime * 1.0e-6,
                                msg->feat_head[0].n_features);
}
*/

static void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, 
        RendererScrollingPlots *self)
{
    if (! strcmp (name, PARAM_NAME_SHOW_LEGEND)) {
        BotGlScrollPlot2dLegendLocation legloc = BOT_GL_SCROLLPLOT2D_HIDDEN;
        if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_SHOW_LEGEND)) {
            legloc = BOT_GL_SCROLLPLOT2D_TOP_RIGHT;
        }
        bot_gl_scrollplot2d_set_show_legend (self->speed_plot, legloc);
        bot_gl_scrollplot2d_set_show_legend (self->pitchroll_plot, legloc);
        bot_gl_scrollplot2d_set_show_legend (self->height_plot, legloc);
        bot_gl_scrollplot2d_set_show_legend (self->heading_plot, legloc);	
        bot_gl_scrollplot2d_set_show_legend (self->depth_plot, legloc);
    }
    bot_viewer_request_redraw(self->viewer);
}

static void 
update_xaxis (RendererScrollingPlots *self, uint64_t utime)
{
    if ((utime < self->max_utime) && 
        (utime > self->max_utime - REDRAW_THRESHOLD_UTIME)) return;

    self->max_utime = utime;
    double timestamp = self->max_utime * 1e-6;
    bot_gl_scrollplot2d_add_point (self->pitchroll_plot, "25000", timestamp, 2500.0);
    bot_gl_scrollplot2d_add_point (self->pitchroll_plot, "0.5", timestamp, 0.5);
    //bot_gl_scrollplot2d_add_point (self->brake_plot, "0.5", timestamp, 0.5);
    //bot_gl_scrollplot2d_add_point (self->depth_plot, "2500", timestamp, 2500.0);

    //bot_gl_scrollplot2d_add_point (self->heading_plot, "N", timestamp, 0);
    //bot_gl_scrollplot2d_add_point (self->heading_plot, "E", timestamp, 90);
    //bot_gl_scrollplot2d_add_point (self->heading_plot, "S", timestamp, 180);
    //bot_gl_scrollplot2d_add_point (self->heading_plot, "W", timestamp, 270);
    //bot_gl_scrollplot2d_add_point (self->depth_plot, "0", timestamp, 0);
    //bot_gl_scrollplot2d_add_point (self->speed_plot, "10", timestamp, 10.0);
    bot_gl_scrollplot2d_add_point (self->height_plot, "1.5m",  timestamp, 1.5);
    bot_gl_scrollplot2d_add_point (self->height_plot, "1m",  timestamp, 1.0);

    //bot_gl_scrollplot2d_add_point (self->heading_plot, "N", timestamp, 0);
    //bot_gl_scrollplot2d_add_point (self->heading_plot, "E", timestamp, 90);
    
    
}

/*
static gboolean
get_speed_update (void *user_data)
{
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
    if (! ctrans_have_pose (self->ctrans)) return TRUE;
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return 0;


    botlcm_pose_t pose;
    ctrans_local_pose (self->ctrans, &pose);

    double speed = sqrt(sq(pose.vel[0]) + sq(pose.vel[1]) + sq(pose.vel[2]));

    bot_gl_scrollplot2d_add_point (self->speed_plot, "pose", 
            pose.utime * 1e-6, speed);
    return TRUE;
}*/
