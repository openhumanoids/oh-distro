#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include <iostream>

#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

//renderers
#include <bot_lcmgl_render/lcmgl_bot_renderer.h>
#include <occ_map/occ_map_renderers.h>


using namespace std;

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

//  if (argc < 2) {
//    fprintf(stderr, "usage: %s <render_plugins>\n", g_path_get_basename(argv[0]));
//    exit(1);
//  }
  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_glib_mainloop_attach_lcm(lcm);

  BotViewer* viewer = bot_viewer_new("LRPlanning Viewer");
  //die cleanly for control-c etc :-)
  bot_gtk_quit_on_interrupt();

  // setup renderers
  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  occ_map_pixel_map_add_renderer_to_viewer(viewer, 1,"SLAM_MAP","Slam Map");
  bot_lcmgl_add_renderer_to_viewer(viewer,lcm, 1);

  //load the renderer params from the config file.
  char *fname = g_build_filename(g_get_user_config_dir(), ".bot-plugin-viewerrc", NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);

  bot_viewer_unref(viewer);
}
