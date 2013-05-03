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

#include <lcmtypes/fovis_update_t.h>
#include <lcmtypes/drc_lcmtypes.h>


#define PARAM_NAME_GRAPH_TIMESPAN "Time span"
#define PARAM_NAME_FREEZE "Freeze"
#define PARAM_NAME_SIZE "Size"
#define PARAM_NAME_RENDER_PITCH "Pitch"
#define PARAM_NAME_RENDER_ROLL "Roll"
#define PARAM_NAME_RENDER_HEIGHT "Height"
#define PARAM_NAME_RENDER_STATS "BW"
#define PARAM_NAME_RENDER_SPEED "Speed"
#define PARAM_NAME_SHOW_LEGEND "Show Legends"

#define REDRAW_THRESHOLD_UTIME 5000000

// max number of points on the scrolling bar:
// was 1000 - but thats only one second of robot utimes
#define MAX_POINTS 3000

#define RENDERER_NAME "KMCL_Scrolling_Plots"

typedef struct _RendererScrollingPlots RendererScrollingPlots;

struct _RendererScrollingPlots {
    BotRenderer renderer;
    BotViewer *viewer;
    lcm_t *lcm;
 //   CTrans *ctrans;

    BotGtkParamWidget    *pw;

    BotGlScrollPlot2d *pitch_plot;
    BotGlScrollPlot2d *roll_plot;
    BotGlScrollPlot2d *height_plot;
    BotGlScrollPlot2d *bandwidth_plot;
    BotGlScrollPlot2d *speed_plot;

    uint64_t      max_utime;
};


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

    bot_gl_scrollplot2d_set_xlim (self->pitch_plot, gs_ts_min, gs_ts_max);
    bot_gl_scrollplot2d_set_xlim (self->roll_plot, gs_ts_min, gs_ts_max);
    bot_gl_scrollplot2d_set_xlim (self->height_plot, gs_ts_min, gs_ts_max); 
    bot_gl_scrollplot2d_set_xlim (self->bandwidth_plot, gs_ts_min, gs_ts_max);
    bot_gl_scrollplot2d_set_xlim (self->speed_plot, gs_ts_min, gs_ts_max);

    int plot_width = bot_gtk_param_widget_get_int (self->pw, PARAM_NAME_SIZE);
    int plot_height = plot_width / 3;

    int x = viewport[2] - plot_width;
    int y = viewport[1];

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_PITCH)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->pitch_plot, 
                x, y, plot_width, plot_height);
        y += plot_height;
    }
    
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_ROLL)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->roll_plot, 
                x, y, plot_width, plot_height);
        y += plot_height;
    }    

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_HEIGHT)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->height_plot, 
                x, y, plot_width, plot_height);
        y += plot_height;
    }

    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_RENDER_STATS)) {
        bot_gl_scrollplot2d_gl_render_at_window_pos (self->bandwidth_plot, 
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
    RendererScrollingPlots *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererScrollingPlots *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
on_bw_stats(const lcm_recv_buf_t * buf, const char *channel, const drc_bandwidth_stats_t *msg, void *user_data){
  RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
  update_xaxis(self,msg->utime);
  if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;

  double elapsed_time = (double) (msg->utime - msg->previous_utime)/1E6 ;
  // 1024 is also used in bot spy:
  double bw_base2robot =  msg->bytes_to_robot /(1024.0* elapsed_time );
  double bw_robot2base=  msg->bytes_from_robot / (1024.0* elapsed_time );
  
  bot_gl_scrollplot2d_add_point (self->bandwidth_plot, "To Robot", 
                                msg->utime * 1.0e-6,
                                bw_base2robot);
  bot_gl_scrollplot2d_add_point (self->bandwidth_plot, "From Robot", 
                                msg->utime * 1.0e-6,
                                bw_robot2base);
  
  bot_gl_scrollplot2d_add_point (self->bandwidth_plot, "32KB",  
                                 msg->utime * 1.0e-6, 
                                 32.0);  
  //printf("got bw_stats msg\n");
  //printf("%f and %f \n", bw_base2robot, bw_robot2base);
}

static void
on_pose_body(const lcm_recv_buf_t * buf, const char *channel, const bot_core_pose_t *msg, void *user_data){
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
    update_xaxis(self,msg->utime);
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;
    double rpy_in[3];
    bot_quat_to_roll_pitch_yaw (msg->orientation, rpy_in) ;
    bot_gl_scrollplot2d_add_point (self->roll_plot, "roll_body", 
                                msg->utime * 1.0e-6, rpy_in[0]*180/M_PI);
    bot_gl_scrollplot2d_add_point (self->pitch_plot, "pitch_body", 
                                msg->utime * 1.0e-6, rpy_in[1]*180/M_PI);
    bot_gl_scrollplot2d_add_point (self->height_plot, "height_body",
                                msg->utime * 1.0e-6, msg->pos[2]);
    
    double speed = sqrt( msg->vel[0]*msg->vel[0] + msg->vel[1]*msg->vel[1] + msg->vel[2]*msg->vel[2] );
    bot_gl_scrollplot2d_add_point (self->speed_plot, "speed_body",
                                msg->utime * 1.0e-6, speed);
    
}

static void
on_pose_head(const lcm_recv_buf_t * buf, const char *channel, const bot_core_pose_t *msg, void *user_data){
    RendererScrollingPlots *self = (RendererScrollingPlots*) user_data;
    update_xaxis(self,msg->utime);
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;
    double rpy_in[3];
    bot_quat_to_roll_pitch_yaw (msg->orientation, rpy_in) ;
    bot_gl_scrollplot2d_add_point (self->roll_plot, "roll_head", 
                                msg->utime * 1.0e-6, rpy_in[0]*180/M_PI);
    bot_gl_scrollplot2d_add_point (self->pitch_plot, "pitch_head", 
                                msg->utime * 1.0e-6, rpy_in[1]*180/M_PI);
    bot_gl_scrollplot2d_add_point (self->height_plot, "height_head",
                                msg->utime * 1.0e-6, msg->pos[2]);

    double speed = sqrt( msg->vel[0]*msg->vel[0] + msg->vel[1]*msg->vel[1] + msg->vel[2]*msg->vel[2] );
    bot_gl_scrollplot2d_add_point (self->speed_plot, "speed_head",
                                msg->utime * 1.0e-6, speed);  
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
      BOT_GTK_PARAM_WIDGET_SLIDER, 0.1, 10, 0.1, 1); // was 12 seconds
  bot_gtk_param_widget_add_booleans (self->pw,
      BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_NAME_FREEZE, 0, NULL);
  bot_gtk_param_widget_add_booleans (self->pw, 0,
      PARAM_NAME_RENDER_PITCH, 1,
      PARAM_NAME_RENDER_ROLL, 1, NULL);
  bot_gtk_param_widget_add_booleans (self->pw, 0,
      PARAM_NAME_RENDER_HEIGHT, 1,
      PARAM_NAME_RENDER_STATS, 1,
      PARAM_NAME_RENDER_SPEED, 1, NULL);  
  bot_gtk_param_widget_add_booleans (self->pw, 0,
      PARAM_NAME_SHOW_LEGEND, 0, NULL);

  g_signal_connect (G_OBJECT (self->pw), "changed",
      G_CALLBACK (on_param_widget_changed), self);


  // save widget modes:
  g_signal_connect (G_OBJECT (viewer), "load-preferences",
      G_CALLBACK (on_load_preferences), self);
  g_signal_connect (G_OBJECT (viewer), "save-preferences",
      G_CALLBACK (on_save_preferences), self);

  // Pitch plot
  self->pitch_plot = bot_gl_scrollplot2d_new ();
  bot_gl_scrollplot2d_set_title        (self->pitch_plot, "Pitch [d]");
  bot_gl_scrollplot2d_set_text_color   (self->pitch_plot, 0.7, 0.7, 0.7, 1);
  bot_gl_scrollplot2d_set_bgcolor      (self->pitch_plot, 0.1, 0.1, 0.1, 0.7);
  bot_gl_scrollplot2d_set_border_color (self->pitch_plot, 1, 1, 1, 0.7);
  bot_gl_scrollplot2d_set_ylim    (self->pitch_plot, 0, 60);
  bot_gl_scrollplot2d_add_plot    (self->pitch_plot, "pitch_body", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->pitch_plot, "pitch_body", 0, 0, 1, 1);
  bot_gl_scrollplot2d_add_plot    (self->pitch_plot, "pitch_head", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->pitch_plot, "pitch_head", 0, 1, 0, 1);

  // Roll plot
  self->roll_plot = bot_gl_scrollplot2d_new ();
  bot_gl_scrollplot2d_set_title        (self->roll_plot, "Roll [d]");
  bot_gl_scrollplot2d_set_text_color   (self->roll_plot, 0.7, 0.7, 0.7, 1);
  bot_gl_scrollplot2d_set_bgcolor      (self->roll_plot, 0.1, 0.1, 0.1, 0.7);
  bot_gl_scrollplot2d_set_border_color (self->roll_plot, 1, 1, 1, 0.7);
  bot_gl_scrollplot2d_set_ylim    (self->roll_plot, -20, 20);
  bot_gl_scrollplot2d_add_plot    (self->roll_plot, "roll_body", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->roll_plot, "roll_body", 0, 0, 1, 1);
  bot_gl_scrollplot2d_add_plot    (self->roll_plot, "roll_head", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->roll_plot, "roll_head", 0, 1, 0, 1);    
  
  self->height_plot = bot_gl_scrollplot2d_new ();
  bot_gl_scrollplot2d_set_title        (self->height_plot, "Height");
  bot_gl_scrollplot2d_set_text_color   (self->height_plot, 0.7, 0.7, 0.7, 1);
  bot_gl_scrollplot2d_set_bgcolor      (self->height_plot, 0.1, 0.1, 0.1, 0.7);
  bot_gl_scrollplot2d_set_border_color (self->height_plot, 1, 1, 1, 0.7);
  bot_gl_scrollplot2d_set_ylim    (self->height_plot, 0, 2);
  bot_gl_scrollplot2d_add_plot    (self->height_plot, "height_body", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->height_plot, "height_body", 0, 0, 1, 1);
  bot_gl_scrollplot2d_add_plot    (self->height_plot, "height_head", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->height_plot, "height_head", 0, 1, 0, 1);

  // PF Stats plot
  self->bandwidth_plot = bot_gl_scrollplot2d_new ();
  bot_gl_scrollplot2d_set_title        (self->bandwidth_plot, "BW Stats");
  bot_gl_scrollplot2d_set_text_color   (self->bandwidth_plot, 0.7, 0.7, 0.7, 1);
  bot_gl_scrollplot2d_set_border_color (self->bandwidth_plot, 1, 1, 1, 0.7);
  bot_gl_scrollplot2d_set_bgcolor (self->bandwidth_plot, 0.1, 0.1, 0.1, 0.7);
  bot_gl_scrollplot2d_set_ylim    (self->bandwidth_plot, 0, 200.0); // was -1 to 1
  bot_gl_scrollplot2d_add_plot    (self->bandwidth_plot, "To Robot", MAX_POINTS);//0.1);
  bot_gl_scrollplot2d_set_color   (self->bandwidth_plot, "To Robot", 0, 0, 1, 1);
  bot_gl_scrollplot2d_add_plot    (self->bandwidth_plot, "From Robot", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->bandwidth_plot, "From Robot", 1, 1, 0, 1);
  bot_gl_scrollplot2d_add_plot    (self->bandwidth_plot, "32KB", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->bandwidth_plot, "32KB", 0, 1, 0, 1);

  // speed plot
  self->speed_plot = bot_gl_scrollplot2d_new ();
  bot_gl_scrollplot2d_set_title        (self->speed_plot, "Speed [m/s]");
  bot_gl_scrollplot2d_set_text_color   (self->speed_plot, 0.7, 0.7, 0.7, 1);
  bot_gl_scrollplot2d_set_border_color (self->speed_plot, 1, 1, 1, 0.7);
  bot_gl_scrollplot2d_set_bgcolor (self->speed_plot, 0.1, 0.1, 0.1, 0.7);
  bot_gl_scrollplot2d_set_ylim    (self->speed_plot, -0.1, 1.0);
  bot_gl_scrollplot2d_add_plot    (self->speed_plot, "zero", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->speed_plot, "zero", 1, 1, 0, 1);
  bot_gl_scrollplot2d_add_plot    (self->speed_plot, "speed_body", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->speed_plot, "speed_body", 0, 0, 1, 1);
  bot_gl_scrollplot2d_add_plot    (self->speed_plot, "speed_head", MAX_POINTS);
  bot_gl_scrollplot2d_set_color   (self->speed_plot, "speed_head", 0, 1, 0, 1);

  // legends?
  BotGlScrollPlot2dLegendLocation legloc = BOT_GL_SCROLLPLOT2D_HIDDEN;
  if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_SHOW_LEGEND)) {
    legloc = BOT_GL_SCROLLPLOT2D_TOP_RIGHT;
  }
  bot_gl_scrollplot2d_set_show_legend (self->speed_plot, legloc);
  bot_gl_scrollplot2d_set_show_legend (self->pitch_plot, legloc);
  bot_gl_scrollplot2d_set_show_legend (self->roll_plot, legloc);
  bot_gl_scrollplot2d_set_show_legend (self->bandwidth_plot, legloc);


  // subscribe to LC messages
  //self->can_decode = can_decode_new (self->lcm);
  drc_bandwidth_stats_t_subscribe(self->lcm,"BW_STATS",on_bw_stats,self);

  bot_core_pose_t_subscribe(self->lcm,"POSE_BODY",on_pose_body,self);
  bot_core_pose_t_subscribe(self->lcm,"POSE_HEAD",on_pose_head,self);

  // periodically pull pose data from CTrans
  //g_timeout_add (30, get_speed_update, self);

  printf("Finished Setting Up Scrolling Plots\n");

  bot_viewer_add_renderer(viewer, &self->renderer, priority);

  return &self->renderer;
}




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
        bot_gl_scrollplot2d_set_show_legend (self->pitch_plot, legloc);
        bot_gl_scrollplot2d_set_show_legend (self->roll_plot, legloc);
        bot_gl_scrollplot2d_set_show_legend (self->height_plot, legloc);
        bot_gl_scrollplot2d_set_show_legend (self->bandwidth_plot, legloc);
    }
    bot_viewer_request_redraw(self->viewer);
}

static void 
update_xaxis (RendererScrollingPlots *self, uint64_t utime)
{
  
    //if ((utime < self->max_utime) && 
    //    (utime > self->max_utime - REDRAW_THRESHOLD_UTIME)) return;

    self->max_utime = utime;
    double timestamp = self->max_utime * 1e-6;
    bot_gl_scrollplot2d_add_point (self->pitch_plot, "25000", timestamp, 2500.0);
    bot_gl_scrollplot2d_add_point (self->pitch_plot, "0.5", timestamp, 0.5);

    bot_gl_scrollplot2d_add_point (self->roll_plot, "25000", timestamp, 2500.0);
    bot_gl_scrollplot2d_add_point (self->roll_plot, "0.5", timestamp, 0.5);
    
    //bot_gl_scrollplot2d_add_point (self->brake_plot, "0.5", timestamp, 0.5);
    //bot_gl_scrollplot2d_add_point (self->depth_plot, "2500", timestamp, 2500.0);

    //bot_gl_scrollplot2d_add_point (self->depth_plot, "0", timestamp, 0);
    //bot_gl_scrollplot2d_add_point (self->speed_plot, "10", timestamp, 10.0);
    bot_gl_scrollplot2d_add_point (self->height_plot, "1.5m",  timestamp, 1.5);
    bot_gl_scrollplot2d_add_point (self->height_plot, "1m",  timestamp, 1.0);
    

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
