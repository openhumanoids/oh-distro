#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include <iostream>

#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <path_util/path_util.h>

//renderers
#include <bot_lcmgl_render/lcmgl_bot_renderer.h>
#include <bot_frames/bot_frames_renderers.h>
#include <laser_utils/renderer_laser.h>
#include <octomap_utils/renderer_octomap.h>
#include <occ_map/occ_map_renderers.h>
#include <mav_mapping_renderers/mav_mapping_renderers.h>

using namespace std;

// TBD - correct location for this code?
static void on_top_view_clicked(GtkToggleToolButton *tb, void *user_data)
{
  BotViewer *self = (BotViewer*) user_data;

  double eye[3];
  double look[3];
  double up[3];
  self->view_handler->get_eye_look(self->view_handler, eye, look, up);
  eye[0] = 0;
  eye[1] = 0;
  eye[2] = 10;
  look[0] = 0;
  look[1] = 0;
  look[2] = 0;
  up[0] = 0;
  up[1] = 10;
  up[2] = 0;
  self->view_handler->set_look_at(self->view_handler, eye, look, up);

  bot_viewer_request_redraw(self);
}

int main(int argc, char *argv[])
{
  gtk_init(&argc, &argv);
  glutInit(&argc, argv);
  g_thread_init(NULL);

  lcm_t * lcm = bot_lcm_get_global(NULL);
  BotParam * param = bot_param_get_global(lcm, 1);
  if (param == NULL) {
    char param_fname[1024];
    sprintf(param_fname, "%s/sensorball_mapping.cfg", getConfigPath());
    fprintf(stderr, "Warning: No param server running, falling back to param file:\n %s\n", param_fname);
    param = bot_param_new_from_file(param_fname);
  }

  BotFrames * frames = bot_frames_get_global(lcm, param);

  bot_glib_mainloop_attach_lcm(lcm);

  BotViewer* viewer = bot_viewer_new("Mapping Viewer");
  //die cleanly for control-c etc :-)
  bot_gtk_quit_on_interrupt();

  // setup renderers
  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  occ_map_pixel_map_add_renderer_to_viewer(viewer, 1, "SLAM_MAP", "Slam Map");
  add_octomap_renderer_to_viewer(viewer, 1, lcm);
  laser_util_add_renderer_to_viewer(viewer, 1, lcm, param, frames);
  bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);
  bot_frames_add_named_renderer_to_viewer(viewer, 1, frames, "BOT_FRAMES");
  mav_mapping_point_cloud_reg_add_renderer_to_viewer(viewer, 1);

  // add custon TOP VIEW button
  GtkWidget *top_view_button;
  top_view_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_ZOOM_FIT);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(top_view_button), "Top View");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(top_view_button), viewer->tips, "Switch to Top View", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(top_view_button), 4);
  gtk_widget_show(top_view_button);
  g_signal_connect(G_OBJECT(top_view_button), "clicked", G_CALLBACK(on_top_view_clicked), viewer);

  on_top_view_clicked(NULL, (void *) viewer);

  //load the renderer params from the config file.
  char *fname = g_build_filename(g_get_user_config_dir(), ".mapping-viewerrc", NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);

  bot_viewer_unref(viewer);
}
