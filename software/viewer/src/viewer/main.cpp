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
#include <renderer_cam_thumb_drc/renderer_cam_thumb_drc.h>
#include <visualization/collections_renderer.hpp>
#include <octomap_utils/renderer_octomap.h>
#include <maps-renderer/maps_renderer.hpp>
#include <data-control-renderer/DataControlRenderer.hpp>
#include <atlas-camera-renderer/CameraRenderer.hpp>
// #include <renderer_annotated_camera/AnnotatedCameraRenderer.hpp>
#include <multisense/multisense_renderer.h>
#include <occ_map/occ_map_renderers.h>
// Individual Renderers:
#include <renderer_drc/renderer_scrollingplots.h>
#include <renderer_drc/renderer_walking.hpp>
#include <renderer_drc/renderer_controller_options.hpp>
#include <renderer_drc/renderer_status.hpp>
#include <renderer_drc/renderer_drcgrid.h>
#include <renderer_drc/renderer_recovery.hpp>
#include <renderer_drc/renderer_bdi.hpp>

// block of renderers
#include <renderer_robot_state/renderer_robot_state.hpp>
#include <renderer_robot_plan/renderer_robot_plan.hpp>
#include <renderer_affordances/renderer_affordances.hpp>
#include <renderer_sticky_feet/renderer_sticky_feet.hpp>
#include <tracker-renderer/TrackerRenderer.hpp>

// mav renderers:
#include <mav_state_est/mav_state_est_renderers.h>
#include <octomap_utils/renderer_octomap.h>
#include <occ_map/occ_map_renderers.h>

#include "udp_util.h"
#include "RendererGroupUtil.hpp"
#include "lcmtypes/drc_plan_control_t.h"
#include "lcmtypes/drc_robot_posture_preset_t.h"
#include <visualization_utils/keyboard_signal_utils.hpp>
#include <visualization_utils/foviation_signal_utils.hpp>
#include <visualization_utils/affordance_utils/aff_trigger_signal_utils.hpp>


using namespace std;
using namespace visualization_utils;

// NOTE (Sisir, 8th Jul 13): We dont want to have individual keyboard event handlers in renderers
// as designed in lib-bot as the key events can be non-unique and the key handlers can conflict. 
// Using boost::signals to create a global keyboard signal. Each renderer creates a corresponding 
// slot to handle global key events. Using a shared ptr to the boost signal. Renderers that require access
// to keyboard signals will receive the signal ref as an argument in their setup function.
KeyboardSignalRef _keyboardSignalRef = KeyboardSignalRef(new KeyboardSignal()); 
AffTriggerSignalsRef _affTriggerSignalsRef = AffTriggerSignalsRef(new AffTriggerSignals()); 
RendererFoviationSignalRef _rendererFoviationSignalRef  = RendererFoviationSignalRef(new RendererFoviationSignal()); 
boost::shared_ptr<RendererFoviationSignalHandler> foviationSignalHndlr;

static void foviationSignalCallback(void *user_data, string renderer_name, bool toggle)
{            
}


static int
logplayer_remote_on_key_press(BotViewer *viewer, BotEventHandler *ehandler,
        const GdkEventKey *event)
{
    int keyval = event->keyval;
    //std::cout << "keyval: " << keyval << "\n";
    
    // emit global keyboard signal, second argument indicates that it is a keypress (if true)
    (*_keyboardSignalRef)(keyval,true); 
    
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
      case SHIFT_L:
      case SHIFT_R:
          //std::cout << "shift pressed\n";
          break;
      default:
          break;
    }
 
  return 0;
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
  eye[2] = 4;
  look[0] = 0;
  look[1] = 0;
  look[2] = 0;
  up[0] = 0;
  up[1] = 4;
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

// TBD - correct location for this?
int start_pose_util_counter=0;
static void on_pose_util_clicked(GtkToggleToolButton *tb, void *user_data)
{
  BotViewer *self = (BotViewer*) user_data;
  if (start_pose_util_counter >0){ // is there a better way than this counter?
    int i = system ("drc-robot-pose-util &> /dev/null &");
  }
  start_pose_util_counter++;
}

static void on_stop_manipulation_clicked(GtkToggleToolButton *tb, void *user_data)
{
  lcm_t * lcm = (lcm_t *) user_data;
  drc_plan_control_t msg;
  msg.utime = bot_timestamp_now();
  msg.control = 0;
  drc_plan_control_t_publish( lcm, "COMMITTED_PLAN_PAUSE", &msg);
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
  
  string config_file = "";
  int network_debug = 0; 
  bool use_additional_renderers = false;
  bool use_multisense_renderer = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(config_file, "c", "config_file","Robot cfg file");
  opt.add(network_debug, "n", "network_debug","Network Debug [0 nothing, 1 feet, 2 plan, 3 state]");
  opt.add(use_multisense_renderer, "m", "multisense","Add multisense renderers");
  opt.add(use_additional_renderers, "a", "additional","Add additional renderers: bot_frames");
  opt.parse();
  std::cout << "config_file: " << config_file << "\n";
  std::cout << "network_debug: " << (int) network_debug << "\n";
  string viewer_title = "MIT DRC Viewer";
  string vis_config_file = ".bot-plugin-robot-drc-viewer";
  
  //todo: comment this section
  gtk_init(&argc, &argv);
  glutInit(&argc, argv);
  g_thread_init(NULL);
  
  lcm_t * lcm;
  lcm= lcm_create("");// bot_lcm_get_global(lcm_url.c_str());
  
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
  //ehandler->key_release = on_key_release;
  bot_viewer_add_event_handler(viewer, ehandler, 0);
  foviationSignalHndlr = boost::shared_ptr<RendererFoviationSignalHandler>(new RendererFoviationSignalHandler(_rendererFoviationSignalRef,foviationSignalCallback));

  // Core Renderers
  drcgrid_add_renderer_to_viewer(viewer, 1, lcm, _keyboardSignalRef);
  bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);
  laser_util_add_renderer_to_viewer(viewer, 1, lcm, bot_param, bot_frames);
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );
  collections_add_renderer_to_viewer(viewer, 1, lcm);
  setup_renderer_robot_state(viewer, 0, lcm,0,_keyboardSignalRef,_affTriggerSignalsRef,_rendererFoviationSignalRef);
  maps_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  //atlas_camera_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  setup_renderer_affordances(viewer, 0, lcm, bot_frames,_keyboardSignalRef,_affTriggerSignalsRef,_rendererFoviationSignalRef);
  setup_renderer_robot_plan(viewer, 0, lcm, 0,_keyboardSignalRef,_affTriggerSignalsRef,_rendererFoviationSignalRef);
  data_control_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  setup_renderer_walking(viewer, 0,lcm,bot_param,bot_frames);
  setup_renderer_sticky_feet(viewer, 0, lcm,bot_param,bot_frames,0);
  setup_renderer_controller_options(viewer, 0, lcm, bot_param, bot_frames);
  status_add_renderer_to_viewer(viewer, 0, lcm);
  add_cam_thumb_drc_renderer_to_viewer(viewer, 0, lcm, bot_param, bot_frames);
  bdi_add_renderer_to_viewer(viewer, 0, lcm);

  if (use_multisense_renderer) {
    multisense_add_renderer_to_viewer(viewer, 0,lcm,bot_frames,"CAMERA_LEFT","CAMERA", bot_param);
  }
  if (use_additional_renderers) {
    bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );
    bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );

    //bot_frames_add_articulated_body_renderer_to_viewer(viewer, 1, bot_param, bot_frames, getModelsPath(), "boxy_renderer");
    //bot_frames_add_articulated_body_renderer_to_viewer(viewer, 1, bot_param, bot_frames, getModelsPath(), "model_renderer");
    add_octomap_renderer_to_viewer(viewer, 1, lcm);
    add_map_measurement_renderer_to_viewer(viewer, 1, lcm, bot_param, bot_frames);
    add_mav_state_est_renderer_to_viewer(viewer, 1, lcm, bot_param, bot_frames);
    occ_map_pixel_map_add_renderer_to_viewer(viewer, 1, "SLAM_MAP", "Slam Map");

    // A second robot renderer: (on a different channel, currently "EST_ROBOT_STATE_COMPRESSED_LOOPBACK
    //setup_renderer_robot_state(viewer, 0, lcm,1,_keyboardSignalRef,_affTriggerSignalsRef,_rendererFoviationSignalRef);
  }

  //--------------    Toolbar Additions
  // add custom TOP VIEW button
  GtkWidget *top_view_button;
  top_view_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_ZOOM_FIT);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(top_view_button), "Top View");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(top_view_button), viewer->tips, "Switch to Top View", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(top_view_button), 3);
  gtk_widget_show(top_view_button);
  g_signal_connect(G_OBJECT(top_view_button), "clicked", G_CALLBACK(on_top_view_clicked), viewer);
  on_top_view_clicked(NULL, (void *) viewer);  

  // add custom START SPY button
  GtkWidget *start_spy_button;
  start_spy_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_FIND);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(start_spy_button), "Bot Spy");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(start_spy_button), viewer->tips, "Launch Bot Spy", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(start_spy_button), 3);
  gtk_widget_show(start_spy_button);
  g_signal_connect(G_OBJECT(start_spy_button), "clicked", G_CALLBACK(on_start_spy_clicked), viewer);
  on_start_spy_clicked(NULL, (void *) viewer);    
  
  // add custom POSE UTIL button
  GtkWidget *start_pose_util_button;
  start_pose_util_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_ORIENTATION_PORTRAIT);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(start_pose_util_button), "Pose Util");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(start_pose_util_button), viewer->tips, "Launch Pose Util", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(start_pose_util_button), 3);
  gtk_widget_show(start_pose_util_button);
  g_signal_connect(G_OBJECT(start_pose_util_button), "clicked", G_CALLBACK(on_pose_util_clicked), viewer);
  on_pose_util_clicked(NULL, (void *) viewer);   
  
  // add custom stop manipulation button
  GtkWidget *stop_manipulation_button;
  stop_manipulation_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_PAUSE);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(stop_manipulation_button), "Pause Manipulation Execution");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(stop_manipulation_button), viewer->tips, "Pause Manipulation Execution", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(stop_manipulation_button), 5);
  gtk_widget_show(stop_manipulation_button);
  g_signal_connect(G_OBJECT(stop_manipulation_button), "clicked", G_CALLBACK(on_stop_manipulation_clicked), lcm);
  on_stop_manipulation_clicked(NULL, (void *) lcm);      
  
  // add custom "collapse all" button
  GtkToolItem *item = gtk_tool_button_new_from_stock (GTK_STOCK_CLEAR);
  gtk_tool_button_set_label (GTK_TOOL_BUTTON (item), "Collapse All");
  gtk_tool_item_set_is_important (GTK_TOOL_ITEM (item), TRUE);
  gtk_tool_item_set_tooltip (GTK_TOOL_ITEM (item), viewer->tips,
                             "Collapse all visible renderers", NULL);
  gtk_toolbar_insert (GTK_TOOLBAR (viewer->toolbar), item, 5);
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
