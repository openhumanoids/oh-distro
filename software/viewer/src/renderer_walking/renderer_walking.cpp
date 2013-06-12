// Renderer for a point-and-click message publisher
// used to send a message to relocalize a robot
// this was orginally part of envoy/renderers
// mfallon aug2011
#include <stdio.h>
#include <stdlib.h>
#include <string>
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

#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>

//#include <visualization/renderer_localize.h>
#include <vector>

#include <lcmtypes/drc_lcmtypes.h>
//#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/bot_core.h>

#include <maps/ViewClient.hpp>
#include <maps/BotWrapper.hpp>
#include <string>
#define RENDERER_NAME "Walking"
#define PARAM_GOAL_SEND "Place New Walking Goal"
#define PARAM_GOAL_UPDATE "Update Current Goal"
#define PARAM_FOLLOW_SPLINE "Footsteps follow spline"
#define PARAM_IGNORE_TERRAIN "Footsteps ignore terrain"
#define PARAM_CRAWLING "Crawling"
#define PARAM_LEADING_FOOT "Leading foot"
// #define PARAM_ALLOW_OPTIMIZATION "Allow optimization"
// #define PARAM_STEP_TIME "Time per step (s)"
#define PARAM_STEP_SPEED "Foot speed (1.5 m/s)"
#define PARAM_STEP_HEIGHT "Step clearance (0.1 m)"
#define PARAM_MU "Coeff. of friction (1.00)"
#define PARAM_MAX_NUM_STEPS "Max. number of steps"
#define PARAM_MIN_NUM_STEPS "Min. number of steps"
#define PARAM_NOM_FORWARD_STEP "Nominal forward step (0.25 m)"
#define PARAM_MAX_FORWARD_STEP "Max forward step (0.5 m)"
#define PARAM_NOM_STEP_WIDTH "Nominal step width (0.26 m)"

typedef enum _leading_foot_t {
  LEADING_FOOT_RIGHT, LEADING_FOOT_LEFT
} leading_foot_t;


#define DRAW_PERSIST_SEC 4
#define VARIANCE_THETA (30.0 * 180.0 / M_PI);
//Not sure why abe has this high a variance for the angle //((2*M_PI)*(2*M_PI))
#define MIN_STD 0.3
#define MAX_STD INFINITY

////////////////// THE FOLLOWING CODE WAS COPIED IN HERE TO AVOID
////////////////// DEPENDENCY WITH THE COMMON_UTILS/GEOM_UTILS POD [MFALLON]
#define GEOM_EPSILON 1e-9

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

#define point3d_as_array(p) ((double*)p)

/* The magic below allows you to use the POINT3D() macro to convert a
 * double[3] to a point3d_t.  gcc is smart -- if you try to cast anything
 * other than a point3d_t or a double[] with this macro, gcc will emit
 * a warning. */
union _point3d_any_t {
  point3d_t point;
  double array[3];
};

typedef point3d_t vec3d_t;

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))


int geom_ray_plane_intersect_3d (const point3d_t *ray_point, const vec3d_t *ray_dir,
    const point3d_t *plane_point, const vec3d_t *plane_normal,
    point3d_t *result, double *u)
{
  double lambda1 = ray_dir->x * plane_normal->x +
      ray_dir->y * plane_normal->y +
      ray_dir->z * plane_normal->z;

  // check for degenerate case where ray is (more or less) parallel to plane
  if (fabs (lambda1) < GEOM_EPSILON) return 0;

  double lambda2 = (plane_point->x - ray_point->x) * plane_normal->x +
      (plane_point->y - ray_point->y) * plane_normal->y +
      (plane_point->z - ray_point->z) * plane_normal->z;
  double v = lambda2 / lambda1;
  result->x = ray_point->x + v * ray_dir->x;
  result->y = ray_point->y + v * ray_dir->y;
  result->z = ray_point->z + v * ray_dir->z;
  if (u) *u = v;
  return 1;
}

int geom_ray_z_plane_intersect_3d(const point3d_t *ray_point, 
    const point3d_t *ray_dir, double plane_z, point2d_t *result_xy)
{
  point3d_t plane_pt = { 0, 0, plane_z};
  point3d_t plane_normal = { 0, 0, 1};
  point3d_t plane_isect_point;
  double plane_point_dist;
  if (!geom_ray_plane_intersect_3d (ray_point, ray_dir, &plane_pt,
      &plane_normal, &plane_isect_point, &plane_point_dist) ||
      plane_point_dist <= 0) {
    return -1;
  }
  result_xy->x = plane_isect_point.x;
  result_xy->y = plane_isect_point.y;
  return 0;
}


////////////////////////////// END OF CODE COPIED IN FROM COMMON_UTILS

struct PerceptionData {
  maps::ViewClient mViewClient;
  maps::BotWrapper::Ptr mBotWrapper;
};

typedef struct _RendererWalking {
  BotRenderer renderer;
  BotEventHandler ehandler;
  BotViewer *viewer;
  lcm_t *lc;

  BotGtkParamWidget *pw;
  
  PerceptionData *perceptionData;

  bool has_walking_msg;
  bool follow_spline;
  bool ignore_terrain;
  bool crawling;
  bool allow_optimization;
  drc_walking_goal_t last_walking_msg;
  
  int dragging;
  bool active;
  point2d_t drag_start_local;
  point2d_t drag_finish_local;

  point2d_t click_pos;
  double support_surface_z;
  double theta;
  double goal_std;
  int max_num_steps;
  int min_num_steps;
  leading_foot_t leading_foot;

  int64_t max_draw_utime;
  double circle_color[3];
  
  // Most recent robot position, rotation and utime
  int64_t robot_utime;
  // int64_t time_per_step_ns;
  double step_speed;
  double step_height;
  double nom_forward_step;
  double max_forward_step;
  double nom_step_width;
  double mu;
  double robot_pos[3];
  double robot_rot[4]; // quaternion in xywz
  

}RendererWalking;

static void
_draw (BotViewer *viewer, BotRenderer *renderer)
{
  RendererWalking *self = (RendererWalking*) renderer;
  int64_t now = bot_timestamp_now();

  if(!self->dragging && now > self->max_draw_utime)
    return;

  //glColor3f(0, 1, 0);
  glColor3f(self->circle_color[0], self->circle_color[1], self->circle_color[2]);
  glPushMatrix();
  glTranslatef(self->click_pos.x, self->click_pos.y, self->support_surface_z);

  glLineWidth(3);

  bot_gl_draw_circle(self->goal_std);

  glBegin(GL_LINE_STRIP);
  glVertex2f(0.0,0.0);
  glVertex2f(self->goal_std*cos(self->theta),self->goal_std*sin(self->theta));
  glEnd();
  glPopMatrix();
}

static void
recompute_2d_goal_pose(RendererWalking *self)
{

  self->click_pos = self->drag_start_local;
  double dx = self->drag_finish_local.x - self->drag_start_local.x;
  double dy = self->drag_finish_local.y - self->drag_start_local.y;

  double theta = atan2(dy,dx);
  self->theta = theta;

  self->goal_std = sqrt(dx*dx + dy*dy);
  if(self->goal_std < MIN_STD)
    self->goal_std = MIN_STD;
  if(self->goal_std > MAX_STD)
    self->goal_std = MAX_STD;
  self->max_draw_utime = bot_timestamp_now() + DRAW_PERSIST_SEC * 1000000;

}

static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererWalking *self = (RendererWalking*) ehandler->user;

  //fprintf(stderr, "Active: %d | Mouse Press : %f,%f\n",self->active, ray_start[0], ray_start[1]);

  self->dragging = 0;

  if(!self->active){
    //fprintf(stderr, "Not Active\n");
    return 0;
  }

  if(event->button != 1){
   // fprintf(stderr,"Wrong Button\n");
    return 0;
  }

  point2d_t click_pt_local;
  float zMean = 0;
  
  int iViewId = DRC_DATA_REQUEST_T_HEIGHT_MAP_SCENE;
  maps::ViewClient::ViewPtr view = self->perceptionData->mViewClient.getView(iViewId);
  if (view != NULL) {
    maps::PointCloud::Ptr cloud = view->getAsPointCloud();
    for (int i = 0; i < cloud->size(); ++i) {
      zMean += cloud->points[i].z;
    }
    if (cloud->size() > 0) {
      zMean /= cloud->size();
    }
  }
      
  if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start),
      POINT3D(ray_dir), zMean, &click_pt_local)) {
    bot_viewer_request_redraw(self->viewer);
    self->active = false;
    return 0;
  }

  Eigen::Vector3f intersectionPt(click_pt_local.x,click_pt_local.y,zMean);
  Eigen::Vector3f dummyNormal(0,0,1);
    
  if (view != NULL) {
    Eigen::Vector3f origin(ray_start[0], ray_start[1], ray_start[2]);
    Eigen::Vector3f direction(ray_dir[0], ray_dir[1], ray_dir[2]);
    if(!view->intersectRay(origin, direction, intersectionPt, dummyNormal))
    {
      intersectionPt<< click_pt_local.x,click_pt_local.y,zMean;
      dummyNormal<< 0,0,1;
    }
  }
  std::cout << "Closest " << intersectionPt.transpose() << std::endl;
  
  self->dragging = 1;

  click_pt_local.x = intersectionPt[0];
  click_pt_local.y = intersectionPt[1];
  self->support_surface_z = intersectionPt[2];

  self->drag_start_local = click_pt_local;
  self->drag_finish_local = click_pt_local;

  recompute_2d_goal_pose(self);

  bot_viewer_request_redraw(self->viewer);
  return 1;
}



static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler,
    const double ray_start[3], const double ray_dir[3],
    const GdkEventButton *event)
{
  RendererWalking *self = (RendererWalking*) ehandler->user;

  if (self->dragging) {
    self->dragging = 0;
  }
  if (self->active) {
    // check drag points and publish

    printf("x,y,t: %f %f %f.    std: %f\n",self->click_pos.x
        ,self->click_pos.y,self->theta,self->goal_std);

    fprintf(stderr,"Walking Button Released => Activate Value : %d\n", self->active);

    drc_walking_goal_t msg;
    msg.utime = self->robot_utime; //bot_timestamp_now();
    msg.max_num_steps = (int32_t) self->max_num_steps;
    msg.min_num_steps = (int32_t) self->min_num_steps;
    msg.robot_name = "atlas"; // this should be set from robot state message

    msg.goal_pos.translation.x = self->click_pos.x;
    msg.goal_pos.translation.y = self->click_pos.y;
    msg.goal_pos.translation.z = 0;
    double rpy[] = {0,0,self->theta};
    double quat_out[4];
    bot_roll_pitch_yaw_to_quat(rpy, quat_out); // its in w,x,y,z format
    msg.goal_pos.rotation.w = quat_out[0];
    msg.goal_pos.rotation.x = quat_out[1];
    msg.goal_pos.rotation.y = quat_out[2];
    msg.goal_pos.rotation.z = quat_out[3];
    msg.is_new_goal = true;
    msg.allow_optimization = self->allow_optimization;
    // msg.time_per_step = self->time_per_step_ns;
    msg.step_speed = self->step_speed;
    msg.nom_step_width = self->nom_step_width;
    msg.nom_forward_step = self->nom_forward_step;
    msg.max_forward_step = self->max_forward_step;
    msg.step_height = self->step_height;
    msg.mu = self->mu;
    msg.follow_spline = self->follow_spline;
    msg.ignore_terrain = self->ignore_terrain;
    msg.crawling = self->crawling;
    if (self->leading_foot == LEADING_FOOT_RIGHT) {
      msg.right_foot_lead = true;
    } else {
      msg.right_foot_lead = false;
    }
    self->has_walking_msg = true;
    self->last_walking_msg = msg;

    std::string channel = msg.crawling ? "CRAWLING_NAV_GOAL" : "WALKING_GOAL";
    fprintf(stderr, ("Sending " + channel + "\n").c_str());
    drc_walking_goal_t_publish(self->lc, channel.c_str(), &msg);
    bot_viewer_set_status_bar_message(self->viewer, ("Sent " + channel).c_str());
    
    self->active = false;

    return 0;
  }


  return 0;
}

static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,
    const double ray_start[3], const double ray_dir[3],
    const GdkEventMotion *event)
{
  RendererWalking *self = (RendererWalking*) ehandler->user;

  if(!self->dragging || !self->active)
    return 0;

  point2d_t drag_pt_local;
  if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start),
      POINT3D(ray_dir), self->support_surface_z, &drag_pt_local)) {
    return 0;
  }
  self->drag_finish_local = drag_pt_local;
  recompute_2d_goal_pose(self);

  bot_viewer_request_redraw(self->viewer);
  return 1;
}

void activate(RendererWalking *self)
{
  self->active = true;
  self->goal_std=0;
}

static int key_press (BotViewer *viewer, BotEventHandler *ehandler, 
    const GdkEventKey *event)
{
  RendererWalking *self = (RendererWalking*) ehandler->user;
  self->max_num_steps = bot_gtk_param_widget_get_int(self->pw, PARAM_MAX_NUM_STEPS);
  self->min_num_steps = bot_gtk_param_widget_get_int(self->pw, PARAM_MIN_NUM_STEPS);
  return 0;
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererWalking *self = (RendererWalking*) user;
  self->max_num_steps = bot_gtk_param_widget_get_int(self->pw, PARAM_MAX_NUM_STEPS);
  self->min_num_steps = bot_gtk_param_widget_get_int(self->pw, PARAM_MIN_NUM_STEPS);
  self->step_speed = bot_gtk_param_widget_get_double(self->pw, PARAM_STEP_SPEED);
  self->nom_step_width = bot_gtk_param_widget_get_double(self->pw, PARAM_NOM_STEP_WIDTH);
  self->nom_forward_step = bot_gtk_param_widget_get_double(self->pw, PARAM_NOM_FORWARD_STEP);
    self->max_forward_step = bot_gtk_param_widget_get_double(self->pw, PARAM_MAX_FORWARD_STEP);
  self->step_height = bot_gtk_param_widget_get_double(self->pw, PARAM_STEP_HEIGHT);
  self->mu = bot_gtk_param_widget_get_double(self->pw, PARAM_MU);
  self->follow_spline = bot_gtk_param_widget_get_bool(self->pw, PARAM_FOLLOW_SPLINE);
  self->ignore_terrain = bot_gtk_param_widget_get_bool(self->pw, PARAM_IGNORE_TERRAIN);
  self->crawling = bot_gtk_param_widget_get_bool(self->pw, PARAM_CRAWLING);
  self->leading_foot = (leading_foot_t) bot_gtk_param_widget_get_enum(self->pw, PARAM_LEADING_FOOT);

  // if (msg_changed) {
    if (self->has_walking_msg) {
      self->last_walking_msg.utime = self->robot_utime; //bot_timestamp_now();
      self->last_walking_msg.max_num_steps = self->max_num_steps;
      self->last_walking_msg.min_num_steps = self->min_num_steps;
      self->last_walking_msg.is_new_goal = false;
      self->last_walking_msg.follow_spline = self->follow_spline;
      self->last_walking_msg.ignore_terrain = self->ignore_terrain;
      self->last_walking_msg.crawling = self->crawling;
      self->last_walking_msg.step_speed = self->step_speed;
      self->last_walking_msg.nom_step_width = self->nom_step_width;
      self->last_walking_msg.nom_forward_step = self->nom_forward_step;
      self->last_walking_msg.max_forward_step = self->max_forward_step;
      self->last_walking_msg.step_height = self->step_height;
      self->last_walking_msg.mu = self->mu;
      self->last_walking_msg.allow_optimization = self->allow_optimization;
      // self->last_walking_msg.time_per_step = self->time_per_step_ns;
      if (self->leading_foot == LEADING_FOOT_RIGHT) {
        self->last_walking_msg.right_foot_lead = true;
      } else {
        self->last_walking_msg.right_foot_lead = false;
      }
    }
  // }

  if(!strcmp(name, PARAM_GOAL_UPDATE)) {
    fprintf(stderr, "\nClicked Update Walking Goal\n");
    if (self->has_walking_msg) {
      fprintf(stderr, "Sending WALKING_GOAL\n");
      drc_walking_goal_t_publish(self->lc, "WALKING_GOAL", &(self->last_walking_msg));
      bot_viewer_set_status_bar_message(self->viewer, "Sent WALKING_GOAL");
    }
  }else if(!strcmp(name, PARAM_GOAL_SEND)) {
    fprintf(stderr,"\nClicked WALKING_GOAL\n");
    //bot_viewer_request_pick (self->viewer, &(self->ehandler));
    activate(self);
  }
}

static void on_est_robot_state (const lcm_recv_buf_t * buf, const char *channel, 
                               const drc_robot_state_t *msg, void *user){
  RendererWalking *self = (RendererWalking*) user;
  
  self->robot_utime =msg->utime;
  self->robot_pos[0] = msg->origin_position.translation.x;
  self->robot_pos[1] = msg->origin_position.translation.y;
  self->robot_pos[2] = msg->origin_position.translation.z;
  self->robot_rot[0] = msg->origin_position.rotation.w;
  self->robot_rot[1] = msg->origin_position.rotation.x;
  self->robot_rot[2] = msg->origin_position.rotation.y;
  self->robot_rot[3] = msg->origin_position.rotation.z;
}

static void
_free (BotRenderer *renderer)
{
  RendererWalking *self = (RendererWalking*) renderer;
  delete self->perceptionData;
  free (renderer);
}

BotRenderer *renderer_walking_new (BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param, BotFrames * frames)
{
  RendererWalking *self = (RendererWalking*) calloc (1, sizeof (RendererWalking));
  self->viewer = viewer;
  self->renderer.draw = _draw;
  self->renderer.destroy = _free;
  self->renderer.name = RENDERER_NAME;
  self->renderer.user = self;
  self->renderer.enabled = 1;

  BotEventHandler *ehandler = &self->ehandler;
  ehandler->name = (char*) RENDERER_NAME;
  ehandler->enabled = 1;
  ehandler->pick_query = NULL;
  ehandler->key_press = key_press;
  ehandler->hover_query = NULL;
  ehandler->mouse_press = mouse_press;
  ehandler->mouse_release = mouse_release;
  ehandler->mouse_motion = mouse_motion;
  ehandler->user = self;

  bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);

  self->circle_color[0] = 0;
  self->circle_color[1] = 1;
  self->circle_color[2] = 0;
  
  
  self->lc = lcm; //globals_get_lcm_full(NULL,1);

  self->has_walking_msg = false;
  self->follow_spline = true;
  self->ignore_terrain = false;
  self->crawling = false;
  self->allow_optimization = false;
  // self->time_per_step_ns = 1.3e9;
  self->step_speed = 1.0; // m/s
  self->nom_step_width = 0.26; // m
  self->nom_forward_step = 0.20; // m
  self->max_forward_step = 0.5; // m
  self->step_height = 0.1; // m

  self->mu = 1.0;
  self->leading_foot = LEADING_FOOT_RIGHT;
  
  self->perceptionData = new PerceptionData();
  self->perceptionData->mBotWrapper.reset(new maps::BotWrapper(lcm,param,frames));
  self->perceptionData->mViewClient.setBotWrapper(self->perceptionData->mBotWrapper);
  self->perceptionData->mViewClient.start();
  
  drc_robot_state_t_subscribe(self->lc,"EST_ROBOT_STATE",on_est_robot_state,self); 

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_GOAL_SEND, NULL);
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_GOAL_UPDATE, NULL);
  bot_gtk_param_widget_add_enum(self->pw, PARAM_LEADING_FOOT, BOT_GTK_PARAM_WIDGET_MENU, self->leading_foot, "Right", LEADING_FOOT_RIGHT, "Left", LEADING_FOOT_LEFT, NULL);
  bot_gtk_param_widget_add_double(self->pw, PARAM_MAX_NUM_STEPS, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 30.0, 1.0, 30.0);  
  bot_gtk_param_widget_add_double(self->pw, PARAM_MIN_NUM_STEPS, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 30.0, 1.0, 0.0);  
  // bot_gtk_param_widget_add_double(self->pw, PARAM_STEP_TIME, BOT_GTK_PARAM_WIDGET_SPINBOX, 1.0, 10, 0.1, ((double)self->time_per_step_ns) / 1e9);  
  bot_gtk_param_widget_add_double(self->pw, PARAM_STEP_SPEED, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.2, 5.0, 0.1, self->step_speed);
  bot_gtk_param_widget_add_double(self->pw, PARAM_STEP_HEIGHT, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.05, 0.5, 0.05, self->step_height);
  bot_gtk_param_widget_add_double(self->pw, PARAM_NOM_FORWARD_STEP, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.05, 1.0, 0.05, self->nom_forward_step);
  bot_gtk_param_widget_add_double(self->pw, PARAM_MAX_FORWARD_STEP, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.05, 1.0, 0.05, self->max_forward_step);
  bot_gtk_param_widget_add_double(self->pw, PARAM_NOM_STEP_WIDTH, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.22, 0.4, 0.01, self->nom_step_width);
  bot_gtk_param_widget_add_double(self->pw, PARAM_MU, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.0, 1.5, 0.05, self->mu);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_FOLLOW_SPLINE, 0, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_IGNORE_TERRAIN, 0, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_CRAWLING, 0, NULL);
  bot_gtk_param_widget_set_bool(self->pw, PARAM_FOLLOW_SPLINE, self->follow_spline);
  bot_gtk_param_widget_set_bool(self->pw, PARAM_IGNORE_TERRAIN, self->ignore_terrain);
  bot_gtk_param_widget_set_bool(self->pw, PARAM_CRAWLING, self->crawling);
  


  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  self->renderer.widget = GTK_WIDGET(self->pw);

  self->active = false;

  return &self->renderer;
}

void setup_renderer_walking(BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param,
    BotFrames * frames)
{
  bot_viewer_add_renderer_on_side(viewer, renderer_walking_new(viewer, render_priority, lcm, param, frames),
      render_priority , 0);
}
