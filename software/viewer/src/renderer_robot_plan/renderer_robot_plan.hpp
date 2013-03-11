#ifndef RENDERER_ROBOTPLAN_HPP
#define RENDERER_ROBOTPLAN_HPP

#include <iostream>
#include <boost/function.hpp>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include "lcmtypes/drc_lcmtypes.h"

#include <GL/gl.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_core/rotations.h>
#include <gdk/gdkkeysyms.h>
#include <Eigen/Dense>

#include "RobotPlanListener.hpp"

using namespace std;
using namespace boost;
using namespace renderer_robot_plan;


typedef struct _RendererRobotPlan
{
  BotRenderer renderer;
  BotViewer          *viewer;
  BotGtkParamWidget *pw;
  boost::shared_ptr<RobotPlanListener> robotPlanListener;
  boost::shared_ptr<lcm::LCM> lcm;
  int64_t max_draw_utime;
  BotEventHandler ehandler;
  bool selection_enabled;
  bool clicked;
  bool visualize_bbox;
  bool use_colormap;
  Eigen::Vector3f ray_start;
  Eigen::Vector3f ray_end;
  std::string* selection;
  uint selected_plan_index;
  uint displayed_plan_index;
  // Our only source of a free running clock:
  int64_t robot_utime;
  GtkWidget *plan_execution_dock;
  
  // Vicon seed planning collection settings:
  int vicon_n_plan_samples;
  double vicon_sample_period;
  int8_t vicon_type;
  
  
} RendererRobotPlan;

void setup_renderer_robot_plan(BotViewer *viewer, int render_priority, lcm_t *lcm);

#endif //RENDERER_ROBOTPLAN_HPP
