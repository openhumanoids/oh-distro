#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include <iostream>

#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <path_util/path_util.h>
#include <ConciseArgs>

//renderers
#include <bot_frames/bot_frames_renderers.h>
#include <maps-renderer/maps_renderer.hpp>

using namespace std;

int main(int argc, char *argv[])
{
  string config_file = "drc_robot.cfg";
  bool use_param_server = false;

  ConciseArgs parser(argc, argv);
  parser.add(config_file, "c", "config-file", "Configuration file for use with the viewer.");
  parser.add(use_param_server, "p", "param-server", "Use param-server instead of a configuration file.  This overrides the -c option.");
  parser.parse();



  gtk_init(&argc, &argv);
  glutInit(&argc, argv);
  g_thread_init(NULL);

  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_glib_mainloop_attach_lcm(lcm);
  BotParam * bot_param;
  if(!use_param_server && config_file.size()) {
    fprintf(stderr,"Reading config from file\n");
    std::string config_path = std::string(getConfigPath()) +'/' + std::string(config_file);
    bot_param = bot_param_new_from_file(config_path.c_str());
    if (bot_param == NULL) {
      std::cerr << "Couldn't get bot param from file %s\n" << config_path << std::endl;
      exit(-1);
    }
  }else {
    bot_param = bot_param_new_from_server(lcm, 0);
    if (bot_param == NULL) {
      fprintf(stderr, "Couldn't get bot param from server.\n");
      return 1;
    }
  }

  BotFrames* bot_frames = bot_frames_new(lcm, bot_param);

  BotViewer* viewer = bot_viewer_new("Maps Viewer");
  //die cleanly for control-c etc :-)
  bot_gtk_quit_on_interrupt();

  // setup renderers
  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  maps_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );

  //load the renderer params from the config file.
  char *fname = g_build_filename(g_get_user_config_dir(), ".bot-plugin-viewerrc", NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);

  bot_viewer_unref(viewer);
}
