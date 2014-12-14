#ifndef RENDERER_ROBOT_STATE_HPP
#define RENDERER_ROBOT_STATE_HPP

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <math.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <iostream>
#include <boost/function.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/drill_control_t.hpp"
#include "lcmtypes/drc/joint_angles_t.hpp"
#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/traj_opt_constraint_t.hpp"
#include "lcmtypes/drc/walking_goal_t.hpp"
#include <bot_core/rotations.h>
#include <Eigen/Dense>
#include <visualization_utils/angles.hpp>
#include <visualization_utils/eigen_kdl_conversions.hpp>
#include <visualization_utils/keyboard_signal_utils.hpp>
#include <visualization_utils/affordance_utils/aff_trigger_signal_utils.hpp>
#include <visualization_utils/foviation_signal_utils.hpp>
#include "RobotStateListener.hpp"

#define PARAM_WIRE "Show BBoxs For Meshes"  
#define PARAM_COLOR_ALPHA "Alpha"

using namespace std;
using namespace boost;
using namespace Eigen;
using namespace visualization_utils;

namespace renderer_robot_state 
{
   
  typedef struct _RobotStateRendererStruc 
  {
    BotRenderer renderer;
    BotViewer          *viewer;
    BotGtkParamWidget *pw;
    boost::shared_ptr<renderer_robot_state::RobotStateListener> robotStateListener;
    boost::shared_ptr<lcm::LCM> lcm;
    RendererFoviationSignalRef _rendererFoviationSignalRef;
    bool _renderer_foviate;
    //BotEventHandler *key_handler;
    BotEventHandler ehandler;
    bool selection_enabled;
    bool clicked;
    bool dragging; 
    bool visualize_bbox;
    
    GtkWidget *teleop_popup;
    GtkWidget *teleop_error_entry;
    double active_res;
    int active_ee;
    double active_angres;

    std::string* selection;
    std::string* marker_selection;
    
    
    Eigen::Vector3f ray_start;
    Eigen::Vector3f ray_end;
    Eigen::Vector3f ray_hit;
    Eigen::Vector3f ray_hit_drag;
    Eigen::Vector3f prev_ray_hit_drag;
    Eigen::Vector3f ray_hit_normal;
    Eigen::Vector3f marker_offset_on_press;// maintains this offset while dragging
    double ray_hit_t;

    // transparency of the model:
    float alpha;
    std::string* trigger_source_otdf_id;
    KDL::Frame T_world_trigger_aff;

     void keyboardSignalCallback(int keyval, bool is_pressed)
    {
      //
    }
  
  } RobotStateRendererStruc;


} // end namespace
  
  
void setup_renderer_robot_state(BotViewer *viewer, int render_priority, lcm_t *lcm, int operation_mode, KeyboardSignalRef signalRef,AffTriggerSignalsRef affTriggerSignalsRef,RendererFoviationSignalRef rendererFoviationSignalRef);
#endif //RENDERER_ROBOT_STATE_HPP
