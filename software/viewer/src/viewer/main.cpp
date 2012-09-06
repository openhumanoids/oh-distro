#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include <iostream>

#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

#include <path_util/path_util.h>

#include <bot_frames/bot_frames_renderers.h>

//imported renderers
#include <bot_lcmgl_render/lcmgl_bot_renderer.h>
#include <laser_utils/renderer_laser.h>
#include <image_utils/renderer_cam_thumb.h>
#include <visualization/collections_renderer.hpp>
#include <octomap_utils/renderer_octomap.h>

// local renderers
#include <renderer_drc/renderer_humanoid.hpp>
#include <renderer_drc/renderer_robot_plan.hpp>
//#include <renderer_drc/renderer_localize.hpp>
#include <renderer_drc/renderer_navigation.hpp>
#include <renderer_drc/renderer_end_effector_goal.hpp>
#include "udp_util.h"

using namespace std;


static int
logplayer_remote_on_key_press(BotViewer *viewer, BotEventHandler *ehandler,
        const GdkEventKey *event)
{
    int keyval = event->keyval;

    switch (keyval)
    {
    case 'P':
    case 'p':
        udp_send_string("127.0.0.1", 53261, "PLAYPAUSETOGGLE");
        break;
    case 'N':
    case 'n':
        udp_send_string("127.0.0.1", 53261, "STEP");
        break;
    case '=':
    case '+':
        udp_send_string("127.0.0.1", 53261, "FASTER");
        break;
    case '_':
    case '-':
        udp_send_string("127.0.0.1", 53261, "SLOWER");
        break;
    case '[':
        udp_send_string("127.0.0.1", 53261, "BACK5");
        break;
    case ']':
        udp_send_string("127.0.0.1", 53261, "FORWARD5");
        break;
    default:
        return 0;
    }

    return 1;
}

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

//  BotParam * param;

  //die cleanly for control-c etc :-)
  bot_gtk_quit_on_interrupt();

  // logplayer controls
  BotEventHandler *ehandler = (BotEventHandler*) calloc(1, sizeof(BotEventHandler));
  ehandler->name = "LogPlayer Remote";
  ehandler->enabled = 1;
  ehandler->key_press = logplayer_remote_on_key_press;
  bot_viewer_add_event_handler(viewer, ehandler, 0);

  // setup renderers
  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);
  laser_util_add_renderer_to_viewer(viewer, 1, lcm, bot_param, bot_frames);
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );
  add_cam_thumb_renderer_to_viewer(viewer, 0, lcm, bot_param, bot_frames);
  setup_renderer_humanoid(viewer, 0, lcm);
  collections_add_renderer_to_viewer(viewer, 1);
  add_octomap_renderer_to_viewer(viewer, 1, lcm);
   //setup_renderer_localize(viewer, 0,lcm);
   setup_renderer_navigation(viewer, 0,lcm);
   setup_renderer_robot_plan(viewer, 0, lcm);
  setup_renderer_end_effector_goal(viewer, 0, lcm);
   
  // load the renderer params from the config file.
  char *fname = g_build_filename(g_get_user_config_dir(), ".bot-plugin-viewerrc", NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);

  bot_viewer_unref(viewer);
}
