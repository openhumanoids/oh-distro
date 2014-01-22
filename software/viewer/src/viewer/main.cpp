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

bool infoviation=false;  
std::vector<std::string> enabled_rendererNames;
std::vector<std::string> input_enabled_rendererNames;
static void initRendererCache(void *user_data)
{
  BotViewer *viewer = (BotViewer*) user_data;
  for (unsigned int ridx = 0; ridx < viewer->renderers->len; ridx++) {
  BotRenderer *renderer = (BotRenderer*)g_ptr_array_index(viewer->renderers, ridx);
  string name(renderer->name);
   if(renderer->enabled){
    enabled_rendererNames.push_back(name);
   }
  }
  for (unsigned int i = 0; i < viewer->event_handlers_sorted->len; ++i) {
    BotEventHandler* handler =
      (BotEventHandler*)g_ptr_array_index(viewer->event_handlers_sorted, i);

    string name(handler->name);
    if(handler->enabled)
     input_enabled_rendererNames.push_back(name);
  }    
}


static void foviationSpecificRenderer(void *user_data, string renderer_name)
{
 
 bool store_renderer_state = false;
  if(!infoviation){
   infoviation= true;
   enabled_rendererNames.clear();
   input_enabled_rendererNames.clear();
   store_renderer_state = true;
  }

  BotViewer *viewer = (BotViewer*) user_data;

  for (unsigned int ridx = 0; ridx < viewer->renderers->len; ridx++) {
    BotRenderer *renderer = (BotRenderer*)g_ptr_array_index(viewer->renderers, ridx);
    string name(renderer->name);
    bool always_enabled_renderers = ((name=="Advanced Grid")||(name=="BOT_FRAMES")||(name=="LCM GL")||(name=="System Status"));
    if(renderer_name=="Affordances & StickyHands/Feet")
    {
	    always_enabled_renderers = (always_enabled_renderers||(name=="Maps"));
    }
    else if(renderer_name=="Footstep Plans")
    {
	    always_enabled_renderers = (always_enabled_renderers||(name=="Maps"));
    }
    else if(renderer_name=="State")
    {
	    always_enabled_renderers = (always_enabled_renderers||(name=="Maps"));
    }    
    else if(renderer_name=="Planning")
    {
	    always_enabled_renderers = (always_enabled_renderers||(name=="Maps"));
    } 
    else if(renderer_name=="Walking")
    {
	    always_enabled_renderers = (always_enabled_renderers||(name=="Footstep Plans"));
    }
    else if(renderer_name=="Data Control")
    {
	    always_enabled_renderers = (always_enabled_renderers||(name=="Maps"));
    }
    
    if((renderer->enabled)&&(store_renderer_state)){
      enabled_rendererNames.push_back(name);
      }

    if((name==renderer_name)||always_enabled_renderers){
      renderer->enabled = 1;
      gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(renderer->cmi), renderer->enabled);
      if(name==renderer_name) {
        renderer->expanded = 1;
        gtk_expander_set_expanded (GTK_EXPANDER (renderer->expander),
                                   renderer->expanded);
      }  
    }
    else{
      renderer->enabled = 0;
      gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(renderer->cmi), renderer->enabled);
//      renderer->expanded = FALSE;
//      if (renderer->expander) {
//        gtk_expander_set_expanded (GTK_EXPANDER (renderer->expander),
//                                   renderer->expanded);
//      }            
    }
   }  
   
  // enable and disable relevant inputs
  for (unsigned int i = 0; i < viewer->event_handlers_sorted->len; ++i) {
    BotEventHandler* handler =
      (BotEventHandler*)g_ptr_array_index(viewer->event_handlers_sorted, i);

    string name(handler->name);
    if((handler->enabled)&&(store_renderer_state))
     input_enabled_rendererNames.push_back(name);
    bool always_enabled_handlers = ((name=="Camera Control")||(name=="LogPlayer Remote"));
    if(renderer_name=="Walking")
    {
	    always_enabled_handlers = (always_enabled_handlers||(name=="Footstep Plans"));
    }
    else if(renderer_name=="Data Control")
    {
	    always_enabled_handlers = (always_enabled_handlers||(name=="Maps"));
    }
    
    if ((renderer_name == handler->name)||always_enabled_handlers) {
        handler->enabled = 1;
        gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(handler->cmi), handler->enabled);
    }
    else 
    {
        handler->enabled = 0;
        gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(handler->cmi), handler->enabled);
    }
  }  
  
}

static void unFoviateRenderers(void *user_data)
{
  BotViewer *viewer = (BotViewer*) user_data;
  infoviation = false;
  
  // enable all renderers
  for (unsigned int ridx = 0; ridx < viewer->renderers->len; ridx++) {
    BotRenderer *renderer = (BotRenderer*)g_ptr_array_index(viewer->renderers, ridx);
    //if renderer name is in enabled_rendererNames, reenable it
    string name(renderer->name);
    std::vector<std::string>::const_iterator found;
    found = std::find (enabled_rendererNames.begin(),enabled_rendererNames.end(), name);
    if (found != enabled_rendererNames.end()) {
      renderer->enabled = 1;
      gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(renderer->cmi), renderer->enabled);
    }
    else
    {
      renderer->enabled = 0;
      gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(renderer->cmi), renderer->enabled);  
    }
   }   
  // enable all inputs
  for (unsigned int i = 0; i < viewer->event_handlers_sorted->len; ++i) {
    BotEventHandler* handler =
    (BotEventHandler*)g_ptr_array_index(viewer->event_handlers_sorted, i);
    //if handler name is in input_enabled_rendererNames, reenable it
    string name(handler->name);
    std::vector<std::string>::const_iterator found;
    found = std::find (input_enabled_rendererNames.begin(),input_enabled_rendererNames.end(), name);
    if (found != input_enabled_rendererNames.end()) {
      handler->enabled = 1;
      gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(handler->cmi), handler->enabled);
    }
    else
    {
      handler->enabled = 0;
      gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(handler->cmi), handler->enabled);
    }
  }   
}

static void foviationSignalCallback(void *user_data, string renderer_name, bool toggle)
{

  BotViewer *viewer = (BotViewer*) user_data;
  if(toggle) 
  {
    cout << "Foviation Requested From: " <<  renderer_name << endl;
    foviationSpecificRenderer(viewer,renderer_name);
  }
  else {
    cout << "UnFoviation Requested From: " << renderer_name << endl;
    unFoviateRenderers(viewer);
  }
             
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
 
  if((keyval>=F1)&&(keyval<=F12)) // Disabling viewer's inherent modification of viewpoint on F4 by consuming the event here.
    return 1;
  else
    return 0;
}

static int
on_key_release(BotViewer *viewer, BotEventHandler *ehandler,
        const GdkEventKey *event)
{
    int keyval = event->keyval;
    // emit global keyboard signal, second argument indicates that it is a keyrelease (if false)
    (*_keyboardSignalRef)(keyval,false); // emit global keyboard signal
    //cout << keyval << endl;

    string foviate_renderer = " "; // Focus view on renderers based on function keys
    if(!infoviation){
      initRendererCache(viewer);
    }
    switch (keyval)
    {
      case SHIFT_L:
      case SHIFT_R:
          //std::cout << "shift released\n";
          break;
      case ESC: 
          unFoviateRenderers(viewer);
         break;  
      case F1:
       foviate_renderer = "Affordances & StickyHands/Feet";
       break;
      case F2:
       foviate_renderer = "Planning";
       break;
      case F3:
       foviate_renderer = "State"; 
       break;
      case F4:
       foviate_renderer = "Walking";
       break; 
      case F5:
       foviate_renderer = "Footstep Plans";
       break; 
      case F6:
       foviate_renderer = "Data Control";
       break;
      case F7:
       foviate_renderer = "Maps";
       break;
      case F8:
       foviate_renderer = "Laser";
       break;
     
    
      default:
          return 0;
    }
    
    if((keyval>=F1)&&(keyval<=F12))
    {
      cout << "\n\nRenderer Foviation Via Function Keys: \n";
      cout << "------------------------------------------\n";
      cout << "ESC-All\n";
      cout << "F1-Affordances & StickyHands/Feet\n";
      cout << "F2-Planning\n";
      cout << "F3-State\n";  
      cout << "F4-Walking\n";
      cout << "F5-Footstep Plans\n";
      cout << "F6-DataControl\n";
      cout << "F7-Maps\n";  
      cout << "F8-Laser\n";
      
    
      foviationSpecificRenderer(viewer,foviate_renderer);              
      bot_viewer_set_status_bar_message(viewer, ("Foviating " + foviate_renderer ).c_str());
      
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


static void on_stop_manipulation_clicked(GtkToggleToolButton *tb, void *user_data)
{
  lcm_t * lcm = (lcm_t *) user_data;
  
  drc_plan_control_t msg;
  msg.utime = bot_timestamp_now();
  msg.control = 0;
  
  drc_plan_control_t_publish( lcm, "COMMITTED_PLAN_PAUSE", &msg);
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
        msg.preset = DRC_ROBOT_POSTURE_PRESET_T_LFTHND_DWN;
        drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
        break;
      case 7:
        msg.preset = DRC_ROBOT_POSTURE_PRESET_T_RGTHND_DWN;
        drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
        break;
      case 8:
        msg.preset = DRC_ROBOT_POSTURE_PRESET_T_LFTHND_INHEADVIEW;
        drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
        break;
      case 9:
        msg.preset = DRC_ROBOT_POSTURE_PRESET_T_RGTHND_INHEADVIEW;
        drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
        break;       
      case 10:
        msg.preset = DRC_ROBOT_POSTURE_PRESET_T_HOSE_MATING;
        drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL",&msg);
   /* case 10:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_SITTING_HNDS_DWN;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break; 
      case 11:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_SITTING_HNDS_UP;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;                          
      case 12:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_PROJECTILE;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;                          
      case 13:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_CROUCHING_HNDS_DWN;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;      
      case 14:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_STANDING_RGTHND_REACH;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;
      case 15:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_STANDING_BDI_FP;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break;                               
      case 16:
        msg.preset = DRC_ROBOT_POSTURE_PRESET_T_PRE_INGRESS;
        drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
        break;
      case 17:
       msg.preset = DRC_ROBOT_POSTURE_PRESET_T_LEFT_HAND_EXTENDED;
       drc_robot_posture_preset_t_publish(lcm, "PRESET_POSTURE_GOAL", &msg);
       break; */
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
  ehandler->key_release = on_key_release;
  bot_viewer_add_event_handler(viewer, ehandler, 0);

  // core renderers
  drcgrid_add_renderer_to_viewer(viewer, 1, lcm, _keyboardSignalRef);
  bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);
  laser_util_add_renderer_to_viewer(viewer, 1, lcm, bot_param, bot_frames);
  bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );

  collections_add_renderer_to_viewer(viewer, 1, lcm);
  
    
  foviationSignalHndlr = boost::shared_ptr<RendererFoviationSignalHandler>(new RendererFoviationSignalHandler(_rendererFoviationSignalRef,foviationSignalCallback));
  
  // Block of Renderers:  
  setup_renderer_robot_state(viewer, 0, lcm,0,_keyboardSignalRef,_affTriggerSignalsRef,_rendererFoviationSignalRef);
  
  // Individual Renderers:
  maps_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  atlas_camera_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);

  setup_renderer_affordances(viewer, 0, lcm, bot_frames,_keyboardSignalRef,_affTriggerSignalsRef,_rendererFoviationSignalRef);
  setup_renderer_robot_plan(viewer, 0, lcm, 0,_keyboardSignalRef,_affTriggerSignalsRef,_rendererFoviationSignalRef);
  data_control_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  setup_renderer_walking(viewer, 0,lcm,bot_param,bot_frames);
  setup_renderer_sticky_feet(viewer, 0, lcm,bot_param,bot_frames,0);
  setup_renderer_controller_options(viewer, 0, lcm, bot_param, bot_frames);

  status_add_renderer_to_viewer(viewer, 0, lcm);

  add_cam_thumb_drc_renderer_to_viewer(viewer, 0, lcm, bot_param, bot_frames);
  // Please don't commit this renderer enabled as it is very heavyweight:
  if (use_multisense_renderer) {
    multisense_add_renderer_to_viewer(viewer, 0,lcm,bot_frames,"CAMERA_LEFT","CAMERA", bot_param);
  }
  if (use_additional_renderers) {
    bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );
    bot_frames_add_renderer_to_viewer(viewer, 1, bot_frames );
  }


  bdi_add_renderer_to_viewer(viewer, 0, lcm);

  /*
  // Various Renderers - disabled since VRC
//  annotated_camera_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
//  scrollingplots_add_renderer_to_viewer(viewer, 0, lcm);
  // bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  // Renderers for Testing Loopback Quality:
  if (network_debug == 1){    
    setup_renderer_sticky_feet(viewer, 0, lcm,bot_param,bot_frames,1); // committed
    setup_renderer_sticky_feet(viewer, 0, lcm,bot_param,bot_frames,2); // loopback
  }else if ( network_debug == 2) { /// DONT RUN THIS WHEN WALKING AS ALL THE PLANS CRASH THE VIEWER:
    setup_renderer_robot_plan(viewer, 0, lcm, 1,_keyboardSignalRef);
    setup_renderer_robot_plan(viewer, 0, lcm, 2,_keyboardSignalRef);
  }else if ( network_debug == 3) {
    setup_renderer_robot_state(viewer, 0, lcm, 1);
    // only one debg version needed
  }
  multisense_add_renderer_to_viewer(viewer, 0,lcm,bot_frames,"CAMERA_LEFT","LIDARSWEEP", bot_param);
  multisense_add_renderer_to_viewer(viewer, 0,lcm,bot_frames,"CAMERA_LEFT","CAMERA_SGBM", bot_param);

  // add_octomap_renderer_to_viewer(viewer, 1, lcm);
  // occ_map_pixel_map_add_renderer_to_viewer_lcm(viewer, 0, lcm, "TERRAIN_DIST_MAP", "PixelMap");
  // setup_renderer_driving(viewer, 0, lcm, bot_param, bot_frames);
  // tracker_renderer_setup(viewer, 0, lcm, bot_param, bot_frames);
  // setup_renderer_recovery(viewer, 0,lcm,bot_param,bot_frames);
  // setup_renderer_crawling_plan(viewer,0, lcm, 0);
  */
  
  //--------------    Toolbar Additions
  // add custom TOP VIEW button
  GtkWidget *top_view_button;
  top_view_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_ZOOM_FIT);
  gtk_widget_set_size_request(top_view_button, 70, 70);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(top_view_button), "Top View");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(top_view_button), viewer->tips, "Switch to Top View", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(top_view_button), 3);
  gtk_widget_show(top_view_button);
  g_signal_connect(G_OBJECT(top_view_button), "clicked", G_CALLBACK(on_top_view_clicked), viewer);
  on_top_view_clicked(NULL, (void *) viewer);  

  
  
  // add custom START SPY button
  GtkWidget *start_spy_button;
  start_spy_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_FIND);
  gtk_widget_set_size_request(start_spy_button, 70, 70);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(start_spy_button), "Bot Spy");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(start_spy_button), viewer->tips, "Launch Bot Spy", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(start_spy_button), 3);
  gtk_widget_show(start_spy_button);
  g_signal_connect(G_OBJECT(start_spy_button), "clicked", G_CALLBACK(on_start_spy_clicked), viewer);
  on_start_spy_clicked(NULL, (void *) viewer);    
  

  // add custom stop manipulation button
  GtkWidget *stop_manipulation_button;
  stop_manipulation_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_MEDIA_PAUSE);
  gtk_widget_set_size_request(stop_manipulation_button, 70, 70);
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
                 
  // add a posture_presets_button (if current broadcasts to all planners and controllers to reset their nominal posture, otherwise sends a preset posture goal for planners.)
  GtkWidget *posture_presets_button;
  posture_presets_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_ORIENTATION_PORTRAIT);
  gtk_widget_set_size_request(posture_presets_button, 70, 70); 
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(posture_presets_button), "Posture Presets");
  //gtk_tool_item_set_is_important (GTK_TOOL_ITEM (posture_presets_button), TRUE);
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(posture_presets_button), viewer->tips, "Update nominal posture(q_nom) across P&C to current posture (or) send a resetting posture goal for pre-determined fixed points", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(posture_presets_button), 6);
  gtk_widget_show(posture_presets_button);
  g_signal_connect(G_OBJECT(posture_presets_button), "clicked", G_CALLBACK(on_posture_presets_clicked), lcm);
 // on_posture_presets_clicked(NULL, (void *) viewer);           
  
  GtkWidget * hbox = gtk_hbox_new (FALSE, 5);
  GtkWidget* vseparator = gtk_vseparator_new ();
  GtkWidget* posture_presets_combo_box=gtk_combo_box_new_text();
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Current" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Fix Left" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Fix Right" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Fix Both" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Both Down" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Both Up" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Left Down" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Right Down" );
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Left In Camera");
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Right In Camera");
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Hose Mating");
  /* Disabled, not safe on real robot
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "SitHndsDn" );//
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "SitHndsUp" );//
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Projectile" );//
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "CrouchHndsDn" ); //
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "StndRHndReach" );//
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "StndBDIFP" );//
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Pre_Ingress");//
  gtk_combo_box_append_text( GTK_COMBO_BOX( posture_presets_combo_box ), "Left_arm_extended");//
  */
  gtk_combo_box_set_active(GTK_COMBO_BOX( posture_presets_combo_box ),(gint) 0);
  gtk_combo_box_set_wrap_width( GTK_COMBO_BOX(posture_presets_combo_box), (gint) 1) ;
  g_signal_connect( G_OBJECT( posture_presets_combo_box ), "changed", G_CALLBACK(on_posture_presets_combo_box_changed ), viewer);

  gtk_box_pack_start (GTK_BOX (hbox), vseparator, FALSE, FALSE, 0);
  gtk_box_pack_end (GTK_BOX (hbox), posture_presets_combo_box, FALSE, FALSE, 0);
  GtkToolItem * toolitem = gtk_tool_item_new ();
  gtk_container_add (GTK_CONTAINER (toolitem), hbox);
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(toolitem), viewer->tips, "Posture Presets", NULL);
  gtk_widget_show_all (GTK_WIDGET (toolitem));
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), toolitem, 7);
  
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
