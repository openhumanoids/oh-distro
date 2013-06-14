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
#include <renderer_drc/renderer_drcgrid.h>
#include <renderer_drc/renderer_recovery.hpp>

// block of renderers
#include <renderer_robot_state/renderer_robot_state.hpp>
#include <renderer_robot_plan/renderer_robot_plan.hpp>
#include <renderer_crawling_plan/renderer_crawling_plan.hpp>
#include <renderer_affordances/renderer_affordances.hpp>
#include <renderer_sticky_feet/renderer_sticky_feet.hpp>
#include <renderer_end_effector_goal/renderer_end_effector_goal.hpp>

#include <tracker-renderer/TrackerRenderer.hpp>


#include "udp_util.h"
#include "RendererGroupUtil.hpp"
#include <lcmtypes/drc_lcmtypes.h>

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

int current_active_posture_preset=0;
static void on_posture_presets_combo_box_changed(GtkWidget* cb, void *user_data)
{
  BotViewer *self = (BotViewer*) user_data;
  gint active;
  active = gtk_combo_box_get_active(GTK_COMBO_BOX(cb));
  current_active_posture_preset = (int)active;
 // std::cout << "posture presets combo_box changed to " << (int)active << "\n";
}

int current_active_controller_mode=1;
static void on_controller_mode_combo_box_changed(GtkWidget* cb, void *user_data)
{
  BotViewer *self = (BotViewer*) user_data;
  gint active;
  active = gtk_combo_box_get_active(GTK_COMBO_BOX(cb));
  current_active_controller_mode = (int)active;
  std::cout << "CONTROLLER MODE variable changed to " << (int)active << "\n";
}


static void on_controller_mode_clicked(GtkToggleToolButton *tb, void *user_data)
{
  lcm_t * lcm = (lcm_t *) user_data;
  std::cout << "CONTROLLER MODE published: " << current_active_controller_mode << "\n";
   
  drc_controller_mode_t msg;
  msg.utime = bot_timestamp_now();
  switch (current_active_controller_mode)                                      
  {
    case 0:
      std::cout << "CONTROLLER MODE published: " << current_active_controller_mode << " [BDI]\n";
      msg.mode = DRC_CONTROLLER_MODE_T_BDI;
      drc_controller_mode_t_publish(lcm,"CONTROLLER_MODE",&msg);
      break;
    case 1:
      std::cout << "CONTROLLER MODE published: " << current_active_controller_mode << " [MIT]\n";
      msg.mode = DRC_CONTROLLER_MODE_T_MIT;
      drc_controller_mode_t_publish(lcm,"CONTROLLER_MODE", &msg);
      break;
    default:
     std::cout << "Unknown preset. Not found in lcmtype";
     break;
  }// end switch case
}

static void on_posture_presets_clicked(GtkToggleToolButton *tb, void *user_data)
{
  lcm_t * lcm = (lcm_t *) user_data;
  std::cout << "Active posture preset: " << current_active_posture_preset << "\n";
   drc_robot_posture_preset_t msg;
   msg.utime = bot_timestamp_now();
   
    switch (current_active_posture_preset)                                      
    {
      case 0:
        msg.preset = DRC_ROBOT_POSTURE_PRESET_T_CURRENT;
        drc_robot_posture_preset_t_publish(lcm, "COMMITTED_POSTURE_PRESET", &msg);
        break;
      case 1:
        msg.preset = DRC_ROBOT_POSTURE_PRESET_T_CURRENT_LFTHND_FIX;
        drc_robot_posture_preset_t_publish(lcm, "COMMITTED_POSTURE_PRESET", &msg);
        break;
      case 2:
        msg.preset = DRC_ROBOT_POSTURE_PRESET_T_CURRENT_RGTHND_FIX;
        drc_robot_posture_preset_t_publish(lcm, "COMMITTED_POSTURE_PRESET", &msg);
        break;
      case 3:
        msg.preset = DRC_ROBOT_POSTURE_PRESET_T_CURRENT_BOTHHNDS_FIX;
        drc_robot_posture_preset_t_publish(lcm, "COMMITTED_POSTURE_PRESET", &msg);
        break;
      case 4:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_STANDING_HNDS_DWN;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;   
      case 5:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_STANDING_HNDS_UP;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break; 
      case 6:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_SITTING_HNDS_DWN;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break; 
      case 7:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_SITTING_HNDS_UP;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;                          
      case 8:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_PROJECTILE;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;                          
      case 9:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_CROUCHING_HNDS_DWN;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;      
      case 10:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_STANDING_RGTHND_REACH;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;                             
      case 11:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_LFTHND_DWN;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;
      case 12:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_RGTHND_DWN;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;
      default:
       std::cout << "Unknown preset. Not found in lcmtype";
       break;
    }// end switch case

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
  int network_debug = 0; 
  ConciseArgs opt(argc, (char**)argv);
  opt.add(config_file, "c", "config_file","Robot cfg file");
  opt.add(role, "r", "role","Role - robot or base");
  opt.add(network_debug, "n", "network_debug","Network Debug [0 nothing, 1 feet, 2 plan, 3 state]");
  opt.parse();
  std::cout << "config_file: " << config_file << "\n";
  std::cout << "role: " << role << "\n";
  std::cout << "network_debug: " << (int) network_debug << "\n";
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
  drcgrid_add_renderer_to_viewer(viewer, 1, lcm);
//  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);
  laser_util_add_renderer_to_viewer(viewer, 1, lcm, bot_param, bot_frames);
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );

  collections_add_renderer_to_viewer(viewer, 1, lcm);
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );

  // Block of Renderers:  
  setup_renderer_affordances(viewer, 0, lcm, bot_frames);
  setup_renderer_robot_state(viewer, 0, lcm,0);
  setup_renderer_robot_plan(viewer, 0, lcm, 0);
  setup_renderer_sticky_feet(viewer, 0, lcm,bot_param,bot_frames,0);
  // Renderers for Testing Loopback Quality:
  if (network_debug == 1){    
    setup_renderer_sticky_feet(viewer, 0, lcm,bot_param,bot_frames,1); // committed
    setup_renderer_sticky_feet(viewer, 0, lcm,bot_param,bot_frames,2); // loopback
  }else if ( network_debug == 2) { /// DONT RUN THIS WHEN WALKING AS ALL THE PLANS CRASH THE VIEWER:
    setup_renderer_robot_plan(viewer, 0, lcm, 1);
    setup_renderer_robot_plan(viewer, 0, lcm, 2);
  }else if ( network_debug == 3) {
    setup_renderer_robot_state(viewer, 0, lcm, 1);
    // only one debg version needed
  }
  
  // Individual Renderers:
  add_octomap_renderer_to_viewer(viewer, 1, lcm);
  maps_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  data_control_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  scrollingplots_add_renderer_to_viewer(viewer, 0, lcm);
  status_add_renderer_to_viewer(viewer, 0, lcm);
  //score_add_renderer_to_viewer(viewer, 0, lcm);
  setup_renderer_driving(viewer, 0, lcm, bot_param, bot_frames);
  setup_renderer_walking(viewer, 0,lcm,bot_param,bot_frames);
  //occ_map_pixel_map_add_renderer_to_viewer_lcm(viewer, 0, lcm, "TERRAIN_DIST_MAP", "PixelMap");

  add_cam_thumb_renderer_to_viewer(viewer, 0, lcm, bot_param, bot_frames);
  multisense_add_renderer_to_viewer(viewer, 0,lcm,bot_frames,"CAMERA", bot_param);

  tracker_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  setup_renderer_recovery(viewer, 0,lcm,bot_param,bot_frames);
  setup_renderer_crawling_plan(viewer,0, lcm, 0);
  
  //--------------    Toolbar Additions
  // add custom TOP VIEW button
  GtkWidget *top_view_button;
  top_view_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_ZOOM_FIT);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(top_view_button), "Top View");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(top_view_button), viewer->tips, "Switch to Top View", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(top_view_button), 4);
  gtk_widget_show(top_view_button);
  g_signal_connect(G_OBJECT(top_view_button), "clicked", G_CALLBACK(on_top_view_clicked), viewer);
  on_top_view_clicked(NULL, (void *) viewer);  

  
  // add custom START SPY button
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
                 
  // add a posture_presets_button (if current broadcasts to all planners and controllers to reset their nominal posture, otherwise sends a preset posture goal for planners.)
  GtkWidget *posture_presets_button;
  posture_presets_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_ORIENTATION_PORTRAIT);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(posture_presets_button), "Posture Presets");
  //gtk_tool_item_set_is_important (GTK_TOOL_ITEM (posture_presets_button), TRUE);
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(posture_presets_button), viewer->tips, "Update nominal posture(q_nom) across P&C to current posture (or) send a resetting posture goal for pre-determined fixed points", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(posture_presets_button), -1);
  gtk_widget_show(posture_presets_button);
  g_signal_connect(G_OBJECT(posture_presets_button), "clicked", G_CALLBACK(on_posture_presets_clicked), lcm);
 // on_posture_presets_clicked(NULL, (void *) viewer);           
  
  GtkWidget * hbox = gtk_hbox_new (FALSE, 5);
  GtkWidget* vseparator = gtk_vseparator_new ();
  GtkWidget* posture_presets_combo_box=gtk_combo_box_new_text();
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Current" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Current_LHndFix" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Current_RHndFix" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Current_BothHndFix" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "StndHndsDn" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "StndHndsUp" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "SitHndsDn" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "SitHndsUp" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Projectile" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "CrouchHndsDn" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "StndRHndReach" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "LHnd_Dwn" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "RHnd_Dwn" );
  gtk_combo_box_set_active(GTK_COMBO_BOX( posture_presets_combo_box ),(gint) 0);
  gtk_combo_box_set_wrap_width( GTK_COMBO_BOX(posture_presets_combo_box), (gint) 1) ;
  g_signal_connect( G_OBJECT( posture_presets_combo_box ), "changed", G_CALLBACK(on_posture_presets_combo_box_changed ), viewer);

  gtk_box_pack_start (GTK_BOX (hbox), vseparator, FALSE, FALSE, 0);
  gtk_box_pack_end (GTK_BOX (hbox), posture_presets_combo_box, FALSE, FALSE, 0);
  GtkToolItem * toolitem = gtk_tool_item_new ();
  gtk_container_add (GTK_CONTAINER (toolitem), hbox);
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(toolitem), viewer->tips, "Posture Presets", NULL);
  gtk_widget_show_all (GTK_WIDGET (toolitem));
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), toolitem, -1);
  
    
// TODO: this will moved to somewhere else  
 GtkWidget *controller_mode_button;
  controller_mode_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_EXECUTE);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(controller_mode_button), "Controller_Mode");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(controller_mode_button), viewer->tips, "Set Controller Mode", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(controller_mode_button), -1);
  gtk_widget_show(controller_mode_button);
  g_signal_connect(G_OBJECT(controller_mode_button), "clicked", G_CALLBACK(on_controller_mode_clicked), lcm);
  
  GtkWidget * hbox2 = gtk_hbox_new (FALSE, 5);
  GtkWidget* vseparator2 = gtk_vseparator_new ();
  GtkWidget* controller_mode_combo_box=gtk_combo_box_new_text();
  gtk_combo_box_append_text( GTK_COMBO_BOX( controller_mode_combo_box ), "BDI" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( controller_mode_combo_box ), "MIT" );
  gtk_combo_box_set_active(GTK_COMBO_BOX( controller_mode_combo_box ),(gint) current_active_controller_mode);
  gtk_combo_box_set_wrap_width( GTK_COMBO_BOX(controller_mode_combo_box), (gint) 1) ;
  g_signal_connect( G_OBJECT( controller_mode_combo_box ), "changed", G_CALLBACK(on_controller_mode_combo_box_changed), viewer);
  

  gtk_box_pack_start (GTK_BOX (hbox2), vseparator2, FALSE, FALSE, 0);
  gtk_box_pack_end (GTK_BOX (hbox2), controller_mode_combo_box, FALSE, FALSE, 0);
  GtkToolItem * toolitem2 = gtk_tool_item_new ();
  gtk_container_add (GTK_CONTAINER (toolitem2), hbox2);
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(toolitem2), viewer->tips, "Cntroller Mode Selection", NULL);
  gtk_widget_show_all (GTK_WIDGET (toolitem2));
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), toolitem2, -1);  

    
  //--------------             

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
