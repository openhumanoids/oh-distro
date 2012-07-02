#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include <iostream>

#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

#include <path_util/path_util.h>

#include <bot_frames/bot_frames_renderers.h>

//renderers
#include <bot_lcmgl_render/lcmgl_bot_renderer.h>
#include <renderer_drc/renderer_humanoid.hpp>

using namespace std;

int main(int argc, char *argv[])
{
  //todo: comment this section
  gtk_init(&argc, &argv);
  glutInit(&argc, argv);
  g_thread_init(NULL);
  
  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_glib_mainloop_attach_lcm(lcm);

  std::string config_name;
  config_name = std::string(getConfigPath()) + "/../../../config/drc_robot.cfg";
  BotParam* bot_param = bot_param_new_from_file(config_name.c_str());
  if (bot_param == NULL) {
    std::cerr << "Couldn't get bot param from file %s\n" << config_name << std::endl;
    exit(-1);
  }
  BotFrames* bot_frames = bot_frames_new(lcm, bot_param);


  BotViewer* viewer = bot_viewer_new("MIT DRC Viewer");

  BotParam * param;

  //die cleanly for control-c etc :-)
  bot_gtk_quit_on_interrupt();

  // setup renderers
  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );

  //---humanoid renderer
  setup_renderer_humanoid(viewer, 0, lcm);
  
  //--------


  bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);

  // load the renderer params from the config file.
  char *fname = g_build_filename(g_get_user_config_dir(), ".bot-plugin-viewerrc", NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);

  bot_viewer_unref(viewer);
}
