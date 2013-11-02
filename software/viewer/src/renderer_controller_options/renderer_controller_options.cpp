#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>

#include <lcmtypes/drc_lcmtypes.h>
//#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/bot_core.h>

#include <string>
#include "renderer_controller_options.hpp"
#define RENDERER_NAME "Drake Controller Options"
#define PARAM_SEND_OPTIONS "Send"
#define PARAM_MAP_MODE "Map mode "

void get_params_from_widget(RendererControllerOptions* self) {
  self->map_command = bot_gtk_param_widget_get_enum(self->pw, PARAM_MAP_MODE);
}


static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererControllerOptions *self = (RendererControllerOptions*) user;
  get_params_from_widget(self);
  publish_options(self);
}

void publish_options(RendererControllerOptions* self) {
  // drc_drake_controller_options_t msg;
  drc_map_controller_command_t msg;
  msg.command = self->map_command;
  fprintf(stderr, "Sending %s \n", "MAP_CONTROLLER_COMMAND");
  drc_map_controller_command_t_publish(self->lc, "MAP_CONTROLLER_COMMAND", &(msg));
  bot_viewer_set_status_bar_message(self->viewer, "Sent MAP_CONTROLLER_COMMAND");
}

static void on_est_robot_state (const lcm_recv_buf_t * buf, const char *channel, 
                               const drc_robot_state_t *msg, void *user){
  RendererControllerOptions *self = (RendererControllerOptions*) user;
  self->robot_utime = msg->utime;
}

static void
_draw(BotViewer *viewer, BotRenderer *renderer)
{
  // intentionally left blank
}

static void
_free (BotRenderer *renderer)
{
  RendererControllerOptions *self = (RendererControllerOptions*) renderer;
  free (renderer);
}

BotRenderer *renderer_controller_options_new (BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param, BotFrames * frames)
{
  RendererControllerOptions *self = (RendererControllerOptions*) calloc (1, sizeof (RendererControllerOptions));
  self->viewer = viewer;
  self->renderer.draw = _draw;
  self->renderer.destroy = _free;
  self->renderer.name = RENDERER_NAME;
  self->renderer.user = self;
  self->renderer.enabled = 1;

  self->map_command = DRC_MAP_CONTROLLER_COMMAND_T_FLAT_GROUND;

  self->lc = lcm; //globals_get_lcm_full(NULL,1);
  
  drc_robot_state_t_subscribe(self->lc,"EST_ROBOT_STATE",on_est_robot_state,self); 

  self->renderer.widget = gtk_alignment_new(0,0.5,1.0,0);

  GtkWidget *box;

  box = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(self->renderer.widget), box);
  gtk_widget_show(box);

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  gtk_box_pack_start(GTK_BOX(box), GTK_WIDGET(self->pw), FALSE, TRUE, 0);
  gtk_widget_show(GTK_WIDGET(self->pw));

  bot_gtk_param_widget_add_enum(self->pw, PARAM_MAP_MODE, BOT_GTK_PARAM_WIDGET_MENU, self->map_command, "Full Heightmap", DRC_MAP_CONTROLLER_COMMAND_T_FULL_HEIGHTMAP, "Flat Ground", DRC_MAP_CONTROLLER_COMMAND_T_FLAT_GROUND, "Z Normals", DRC_MAP_CONTROLLER_COMMAND_T_Z_NORMALS, NULL);
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_OPTIONS, NULL);

  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);

  return &self->renderer;
}

void setup_renderer_controller_options(BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param,
    BotFrames * frames)
{
  bot_viewer_add_renderer_on_side(viewer, renderer_controller_options_new(viewer, render_priority, lcm, param, frames),
      render_priority , 0);
}
