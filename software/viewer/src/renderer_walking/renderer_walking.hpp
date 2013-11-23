#ifndef __renderer_walking_h__
#define ___renderer_walking_h__

#include <maps/ViewClient.hpp>
#include <maps/BotWrapper.hpp>

/**
 * @defgroup scrollingplotsBotRenderer scrollingplotsBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include renderer-walking/renderer_walking.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs renderer-walking`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))

struct PerceptionData {
  maps::ViewClient mViewClient;
  maps::BotWrapper::Ptr mBotWrapper;
};

typedef enum _path_t {
  PATH_SPLINE, PATH_NO_SPLINE, PATH_AUTO
} path_t;

typedef enum _walking_mode_t {
  WALKING_TYPICAL, WALKING_DRAKE_FAST, WALKING_MUD, WALKING_CRAWLING, WALKING_TURN_CRAWLING, WALKING_BDI, STEPPING_BDI, STEPPING_BDI_FINE, WALKING_LADDER, STEPPING_BDI_OBSTACLES, WALKING_BDI_INFINITE
} walking_mode_t;

typedef enum _behavior_t {
  BEHAVIOR_WALKING, BEHAVIOR_CRAWLING, BEHAVIOR_BDI_WALKING, BEHAVIOR_BDI_STEPPING
} behavior_t;

typedef enum _walking_goal_type_t {
  GOAL_TYPE_CENTER, GOAL_TYPE_RIGHT_FOOT, GOAL_TYPE_LEFT_FOOT
} walking_goal_type_t;

typedef enum _bdi_toe_off_t {
  BDI_TOE_OFF_DISABLE, BDI_TOE_OFF_ENABLE, BDI_TOE_OFF_FORCE_ENABLE
} bdi_toe_off_t;

// ===== 2 dimensional structure =====
#ifndef _point2d_t_h
typedef struct _point2d {
  double x;
  double y;
} point2d_t;
#endif

// ===== 3 dimensional strucutres =====
// double 
typedef struct _point3d {
  double x;
  double y;
  double z;
} point3d_t;


typedef struct _RendererWalking {
  BotRenderer renderer;
  BotEventHandler ehandler;
  BotViewer *viewer;
  lcm_t *lc;

  // BotGtkParamWidget *pw;
  BotGtkParamWidget *main_pw;
  BotGtkParamWidget *bdi_pw;
  BotGtkParamWidget *drake_pw;
  BotGtkParamWidget *lead_foot_pw;
  BotGtkParamWidget *map_mode_pw;
  BotGtkParamWidget *ignore_terrain_pw;
  BotGtkParamWidget *path_pw;
  GtkWidget *bdi_feedback_table;
  GtkWidget *bdi_label_step_duration;
  GtkWidget *bdi_label_sway_duration;
  GtkWidget *bdi_label_step_height;
  GtkWidget *bdi_label_lift_height;
  GtkWidget *bdi_label_toe_off;
  GtkWidget *bdi_label_knee_nominal;
  GtkWidget *bdi_label_max_body_accel;
  GtkWidget *bdi_label_max_foot_vel;
  GtkWidget *bdi_label_sway_end_dist;
  GtkWidget *bdi_label_step_end_dist;
  GtkWidget *bdi_table;
  
  // PerceptionData *perceptionData;

  path_t path;
  bool ignore_terrain;
  behavior_t behavior;
  walking_goal_type_t goal_type;
  bool allow_optimization;
  bool force_to_sticky_feet;
  bool velocity_based_steps;
  
  int dragging;
  bool active;
  point2d_t drag_start_local;
  point2d_t drag_finish_local;
  point2d_t click_pos;
  point2d_t goal_pos;
  double goal_yaw;

  double support_surface_z;

  double goal_std;

  int max_num_steps;
  int min_num_steps;
  int leading_foot;
  double step_speed;
  double step_height;
  double nom_forward_step;
  double max_forward_step;
  double min_step_width;
  double nom_step_width;
  double max_step_width;
  double mu;
  int walking_settings;
  float bdi_step_duration;
  float bdi_sway_duration;
  float bdi_lift_height;
  float bdi_max_body_accel;
  float bdi_max_foot_vel;
  float bdi_sway_end_dist;
  float bdi_step_end_dist;
  bdi_toe_off_t bdi_toe_off;
  float bdi_knee_nominal;

  int8_t map_command;

  int64_t max_draw_utime;
  double circle_color[3];
  
  // Most recent robot position, rotation and utime
  int64_t robot_utime;
  double robot_pos[3];
  double robot_rot[4]; // quaternion in xywz

  double height_ground;
  
}RendererWalking;

void setup_renderer_walking(BotViewer *viewer, int render_priority, lcm_t* lcm, BotParam * param,
    BotFrames * frames);

void publish_simple_nav(RendererWalking* self, double x, double y, double yaw);
void set_default_params(RendererWalking* self, int mode);
// void set_view_params(RendererWalking* self);
void get_params_from_widget(RendererWalking* self);
void publish_walking_goal(RendererWalking* self, bool is_new);
void publish_walking_opts(RendererWalking* self);

/**
 * @}
 */


#endif
