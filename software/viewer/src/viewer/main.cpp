#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include <getopt.h>
#include <iostream>

#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <path_util/path_util.h>
#include <ConciseArgs>

#include <bot_frames/bot_frames_renderers.h>

//imported renderers
#include <bot_lcmgl_render/lcmgl_bot_renderer.h>
#include <laser_utils/renderer_laser.h>
#include <image_utils/renderer_cam_thumb.h>
#include <visualization/collections_renderer.hpp>
#include <octomap_utils/renderer_octomap.h>
#include <renderer_heightmap/renderer_heightmap.hpp>
#include <renderer_maps/renderer_maps.hpp>

// Individual Renderers:
#include <renderer_drc/renderer_scrollingplots.h>
#include <renderer_drc/renderer_driving.hpp>
#include <renderer_drc/renderer_manipulation.hpp>
#include <renderer_drc/renderer_walking.hpp>

// block of renderers
#include <renderer_robot_state/renderer_robot_state.hpp>
#include <renderer_robot_plan/renderer_robot_plan.hpp>
#include <renderer_affordances/renderer_affordances.hpp>
#include <renderer_end_effector_goal/renderer_end_effector_goal.hpp>


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
  setlinebuf(stdout);
  
  string config_file = "";
  string role = "robot";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(config_file, "c", "config_file","Robot cfg file");
  opt.add(role, "r", "role","Role - robot or base");
  opt.parse();
  std::cout << "config_file: " << config_file << "\n";
  std::cout << "role: " << role << "\n";
  
  //todo: comment this section
  gtk_init(&argc, &argv);
  glutInit(&argc, argv);
  g_thread_init(NULL);
  
  lcm_t * lcm;
  string viewer_title="";  
  if(role.compare("robot") == 0){
     lcm= bot_lcm_get_global(NULL);
     viewer_title = "(Robot) MIT DRC Viewer";
  }else if(role.compare("base") == 0){
     string lcm_url = "udpm://239.255.12.68:1268?ttl=1";
     lcm= lcm_create(lcm_url.c_str());// bot_lcm_get_global(lcm_url.c_str());
     viewer_title = "(Base) MIT DRC Viewer";
  }else{
    std::cout << "DRC Viewer role not understood, choose: robot or base\n";
    return 1;
  }
  
  bot_glib_mainloop_attach_lcm(lcm);
  BotParam * bot_param;
  if(config_file.size()) {
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
  BotViewer* viewer = bot_viewer_new(viewer_title.c_str());

  //die cleanly for control-c etc :-)
  bot_gtk_quit_on_interrupt();

  // logplayer controls
  BotEventHandler *ehandler = (BotEventHandler*) calloc(1, sizeof(BotEventHandler));
  ehandler->name = "LogPlayer Remote";
  ehandler->enabled = 1;
  ehandler->key_press = logplayer_remote_on_key_press;
  bot_viewer_add_event_handler(viewer, ehandler, 0);

  // core renderers
  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);
  laser_util_add_renderer_to_viewer(viewer, 1, lcm, bot_param, bot_frames);
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );
  collections_add_renderer_to_viewer(viewer, 1, lcm);

  // Block of Renderers:  
  setup_renderer_robot_state(viewer, 0, lcm);
  setup_renderer_robot_plan(viewer, 0, lcm);
  setup_renderer_affordances(viewer, 0, lcm);
  setup_renderer_end_effector_goal(viewer, 0, lcm);

  // Individual Renderers:
  add_octomap_renderer_to_viewer(viewer, 1, lcm);
  heightmap_add_renderer_to_viewer(viewer, 0, lcm, bot_param, bot_frames);
  // disabled for now:
  //maps_add_renderer_to_viewer(viewer, 0, lcm, bot_param, bot_frames);
  scrollingplots_add_renderer_to_viewer(viewer, 0, lcm);
  setup_renderer_manipulation(viewer, 0,lcm);
  setup_renderer_driving(viewer, 0, lcm, bot_param, bot_frames);
  setup_renderer_walking(viewer, 0,lcm);

  add_cam_thumb_renderer_to_viewer(viewer, 0, lcm, bot_param, bot_frames);
  
  // add custon TOP VIEW button
  GtkWidget *top_view_button;
  top_view_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_ZOOM_FIT);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(top_view_button), "Top View");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(top_view_button), viewer->tips, "Switch to Top View", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(top_view_button), 4);
  gtk_widget_show(top_view_button);
  g_signal_connect(G_OBJECT(top_view_button), "clicked", G_CALLBACK(on_top_view_clicked), viewer);

  on_top_view_clicked(NULL, (void *) viewer);  
  
  // load the renderer params from the config file.
  char *fname = g_build_filename(g_get_user_config_dir(), ".bot-plugin-drc-viewer", NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);
  bot_viewer_unref(viewer);
}
