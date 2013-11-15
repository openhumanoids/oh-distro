#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include <iostream>

#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

//renderers
#include <renderer_cam_thumb_drc/renderer_cam_thumb_drc.h>

using namespace std;

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
  BotParam * bot_param;

    bot_param = bot_param_new_from_server(lcm, 0);
    if (bot_param == NULL) {
      fprintf(stderr, "Couldn't get bot param from server.\n");
      return 1;
    }

  BotFrames* bot_frames = bot_frames_new(lcm, bot_param);


  BotViewer* viewer = bot_viewer_new("Camera Viewer");
  //die cleanly for control-c etc :-)
  bot_gtk_quit_on_interrupt();

  // setup renderers
  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  add_cam_thumb_drc_renderer_to_viewer(viewer, 0, lcm, bot_param, bot_frames);

  //load the renderer params from the config file.
  char *fname = g_build_filename(g_get_user_config_dir(), ".bot-plugin-viewerrc", NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);

  bot_viewer_unref(viewer);
}
