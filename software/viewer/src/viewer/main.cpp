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
#include <renderer_maps/MapsRenderer.hpp>
#include <renderer_data_control/DataControlRenderer.hpp>
#include <multisense/multisense_renderer.h>
#include <occ_map/occ_map_renderers.h>
// Individual Renderers:
#include <renderer_drc/renderer_scrollingplots.h>
#include <renderer_drc/renderer_driving.hpp>
#include <renderer_drc/renderer_walking.hpp>
#include <renderer_drc/renderer_status.hpp>

// block of renderers
#include <renderer_robot_state/renderer_robot_state.hpp>
#include <renderer_robot_plan/renderer_robot_plan.hpp>
#include <renderer_affordances/renderer_affordances.hpp>
#include <renderer_sticky_feet/renderer_sticky_feet.hpp>
#include <renderer_end_effector_goal/renderer_end_effector_goal.hpp>


#include "udp_util.h"
#include "RendererGroupUtil.hpp"

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


static void on_collapse_all_clicked(GtkToggleToolButton *tb, void *user_data)
{
  BotViewer *viewer = (BotViewer*) user_data;
  for (unsigned int i = 0; i < viewer->renderers->len; ++i) {
    BotRenderer* renderer =
      (BotRenderer*)g_ptr_array_index(viewer->renderers, i);

    renderer->expanded = FALSE;
    if (renderer->expander) {
      gtk_expander_set_expanded (GTK_EXPANDER (renderer->expander),
                                 renderer->expanded);
    }
  }
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

// TBD - correct location for this?
int start_spy_counter=0;
static void on_start_spy_clicked(GtkToggleToolButton *tb, void *user_data)
{
  BotViewer *self = (BotViewer*) user_data;
  if (start_spy_counter >0){ // is there a better way than this counter?
    int i = system ("bot-spy &> /dev/null &");
  }
  start_spy_counter++;
}


static void
destroy_renderers (BotViewer *viewer)
{
  for (unsigned int ridx = 0; ridx < viewer->renderers->len; ridx++) {
    BotRenderer *renderer =
      (BotRenderer*)g_ptr_array_index(viewer->renderers, ridx);
    if (renderer && renderer->destroy) {
      std::cout << "Destroying renderer \"" << renderer->name <<
        "\"..." << std::flush;
      renderer->destroy(renderer);
      std::cout << "Done" << std::endl;
    }
  }

  g_ptr_array_free(viewer->renderers, TRUE);
}

int main(int argc, char *argv[])
{
  setlinebuf(stdout);
  
  string config_file = ""; // leave this empty so force viewer to get it from the param server
  string role = "robot";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(config_file, "c", "config_file","Robot cfg file");
  opt.add(role, "r", "role","Role - robot or base");
  opt.parse();
  std::cout << "config_file: " << config_file << "\n";
  std::cout << "role: " << role << "\n";
  string viewer_title = "(" + role + ") MIT DRC Viewer";
  string vis_config_file = ".bot-plugin-"+ role +"-drc-viewer";
  
  //todo: comment this section
  gtk_init(&argc, &argv);
  glutInit(&argc, argv);
  g_thread_init(NULL);

  
  string lcm_url="";
  std::string role_upper;
  for(short i = 0; i < role.size(); ++i)
     role_upper+= (std::toupper(role[i]));
  if((role.compare("robot") == 0) || (role.compare("base") == 0) ){
    for(short i = 0; i < role_upper.size(); ++i)
       role_upper[i] = (std::toupper(role_upper[i]));
    string env_variable_name = string("LCM_URL_DRC_" + role_upper); 
    char* env_variable;
    env_variable = getenv (env_variable_name.c_str());
    if (env_variable!=NULL){
      //printf ("The env_variable is: %s\n",env_variable);      
      lcm_url = string(env_variable);
    }else{
      std::cout << env_variable_name << " environment variable has not been set ["<< lcm_url <<"]\n";     
      exit(-1);
    }
  }else{
    std::cout << "Role not understood, choose: robot or base\n";
    return 1;
  }   
  
  lcm_t * lcm;
  lcm= lcm_create(lcm_url.c_str());// bot_lcm_get_global(lcm_url.c_str());
  
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
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );

  // Block of Renderers:  
  setup_renderer_robot_state(viewer, 0, lcm);
  setup_renderer_robot_plan(viewer, 0, lcm);
  setup_renderer_affordances(viewer, 0, lcm);
  setup_renderer_sticky_feet(viewer, 0, lcm,bot_param,bot_frames);
  setup_renderer_end_effector_goal(viewer, 0, lcm);

  // Individual Renderers:
  add_octomap_renderer_to_viewer(viewer, 1, lcm);
  maps_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  data_control_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  scrollingplots_add_renderer_to_viewer(viewer, 0, lcm);
  status_add_renderer_to_viewer(viewer, 0, lcm);
  setup_renderer_driving(viewer, 0, lcm, bot_param, bot_frames);
  setup_renderer_walking(viewer, 0,lcm,bot_param,bot_frames);
  occ_map_pixel_map_add_renderer_to_viewer_lcm(viewer, 0, lcm, "TERRAIN_DIST_MAP", "PixelMap");

  add_cam_thumb_renderer_to_viewer(viewer, 0, lcm, bot_param, bot_frames);
  multisense_add_renderer_to_viewer(viewer, 0,lcm,bot_frames,"CAMERA", bot_param);

  // add custom TOP VIEW button
  GtkWidget *top_view_button;
  top_view_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_ZOOM_FIT);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(top_view_button), "Top View");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(top_view_button), viewer->tips, "Switch to Top View", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(top_view_button), 4);
  gtk_widget_show(top_view_button);
  g_signal_connect(G_OBJECT(top_view_button), "clicked", G_CALLBACK(on_top_view_clicked), viewer);
  on_top_view_clicked(NULL, (void *) viewer);  

  
  // add custom TOP VIEW button
  GtkWidget *start_spy_button;
  start_spy_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_FIND);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(start_spy_button), "Bot Spy");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(start_spy_button), viewer->tips, "Launch Bot Spy", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(start_spy_button), 4);
  gtk_widget_show(start_spy_button);
  g_signal_connect(G_OBJECT(start_spy_button), "clicked", G_CALLBACK(on_start_spy_clicked), viewer);
  on_start_spy_clicked(NULL, (void *) viewer);    
  
  // add custom "collapse all" button
  GtkToolItem *item = gtk_tool_button_new_from_stock (GTK_STOCK_CLEAR);
  gtk_tool_button_set_label (GTK_TOOL_BUTTON (item), "Collapse All");
  gtk_tool_item_set_is_important (GTK_TOOL_ITEM (item), TRUE);
  gtk_tool_item_set_tooltip (GTK_TOOL_ITEM (item), viewer->tips,
                             "Collapse all visible renderers", NULL);
  gtk_toolbar_insert (GTK_TOOLBAR (viewer->toolbar), item, -1);
  gtk_widget_show (GTK_WIDGET (item));
  g_signal_connect (G_OBJECT (item), "clicked", 
                    G_CALLBACK (on_collapse_all_clicked), viewer);


  // add custom renderer groups menu
  RendererGroupUtil groupUtil(viewer, bot_param);
  groupUtil.setup();
  
  // load the renderer params from the config file.
  char *fname = g_build_filename(g_get_user_config_dir(), vis_config_file.c_str() , NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);

  // clean up all renderers
  destroy_renderers(viewer);

  // remove reference
  bot_viewer_unref(viewer);
}
