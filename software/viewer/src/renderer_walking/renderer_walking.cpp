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

#include <string>
#include "renderer_walking.hpp"
#define RENDERER_NAME "Walking"
#define PARAM_GOAL_SEND "Place New Walking Goal"
#define PARAM_GO_FORWARD "Go Forward"
#define PARAM_GOAL_UPDATE "Update Current Goal"
#define PARAM_FOLLOW_SPLINE "Footsteps follow spline"
#define PARAM_IGNORE_TERRAIN "Footsteps ignore terrain"
#define PARAM_BEHAVIOR "Behavior"
#define PARAM_GOAL_TYPE "Goal sets pose of: "
#define PARAM_LEADING_FOOT "Leading foot"
#define PARAM_STEP_SPEED "Foot speed (1.5 m/s)"
#define PARAM_STEP_HEIGHT "Step clearance (0.1 m)"
#define PARAM_MU "Coeff. of friction (1.00)"
#define PARAM_MAX_NUM_STEPS "Max. number of steps"
#define PARAM_MIN_NUM_STEPS "Min. number of steps"
#define PARAM_NOM_FORWARD_STEP "Nominal forward step (0.25 m)"
#define PARAM_MAX_FORWARD_STEP "Max forward step (0.5 m)"
#define PARAM_NOM_STEP_WIDTH "Nominal step width (0.26 m)"

#define WALKING_MODE "Preset"


#define DRAW_PERSIST_SEC 4
#define VARIANCE_THETA (30.0 * 180.0 / M_PI);
//Not sure why abe has this high a variance for the angle //((2*M_PI)*(2*M_PI))
#define MIN_STD 0.3
#define MAX_STD INFINITY

////////////////// THE FOLLOWING CODE WAS COPIED IN HERE TO AVOID
////////////////// DEPENDENCY WITH THE COMMON_UTILS/GEOM_UTILS POD [MFALLON]
#define GEOM_EPSILON 1e-9

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
  glVertex2f(self->goal_std*cos(self->goal_yaw),self->goal_std*sin(self->goal_yaw));
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
  self->goal_yaw = theta;

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
        ,self->click_pos.y,self->goal_yaw,self->goal_std);

    fprintf(stderr,"Walking Button Released => Activate Value : %d\n", self->active);

    self->has_walking_msg = true;
    self->goal_pos.x = self->click_pos.x;
    self->goal_pos.y = self->click_pos.y;

    publish_walking_goal(self, TRUE);
    
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
  

  if(!strcmp(name, WALKING_MODE)) {
    int mode = (walking_mode_t) bot_gtk_param_widget_get_enum(self->pw, WALKING_MODE);
    if (mode == WALKING_TYPICAL){
      std::cout << "Using preset mode: Walking\n";
      bot_gtk_param_widget_set_int(self->pw, PARAM_MAX_NUM_STEPS, 30);  
      bot_gtk_param_widget_set_int(self->pw, PARAM_MIN_NUM_STEPS, 0);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_STEP_SPEED, 1.0);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_STEP_HEIGHT, 0.1);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_NOM_FORWARD_STEP, 0.2);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_MAX_FORWARD_STEP, 0.5);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_NOM_STEP_WIDTH, 0.26);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_MU, 1.0);  
      bot_gtk_param_widget_set_enum(self->pw, PARAM_BEHAVIOR, BEHAVIOR_WALKING);
    }else if (mode == WALKING_MUD){
      std::cout << "Using preset mode: Mud\n";
      bot_gtk_param_widget_set_int(self->pw, PARAM_MAX_NUM_STEPS, 6);  
      bot_gtk_param_widget_set_int(self->pw, PARAM_MIN_NUM_STEPS, 0);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_STEP_SPEED, 0.5);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_STEP_HEIGHT, 0.05);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_NOM_FORWARD_STEP, 0.05);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_MAX_FORWARD_STEP, 0.15);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_NOM_STEP_WIDTH, 0.26);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_MU, 1.0);  
      bot_gtk_param_widget_set_enum(self->pw, PARAM_BEHAVIOR, BEHAVIOR_WALKING);
    }else if (mode == WALKING_CRAWLING){
      std::cout << "Using preset mode: Crawling\n";      
      bot_gtk_param_widget_set_int(self->pw, PARAM_MAX_NUM_STEPS, 30);  
      bot_gtk_param_widget_set_int(self->pw, PARAM_MIN_NUM_STEPS, 0);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_STEP_SPEED, 0.8);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_STEP_HEIGHT, 0.3);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_NOM_FORWARD_STEP, 0.3);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_MAX_FORWARD_STEP, 0.5);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_NOM_STEP_WIDTH, 0.26);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_MU, 0.2);  
      bot_gtk_param_widget_set_enum(self->pw, PARAM_BEHAVIOR, BEHAVIOR_CRAWLING);
    }else if (mode == WALKING_TURN_CRAWLING){
      std::cout << "Using preset mode: Turn Crawling\n";            
      bot_gtk_param_widget_set_int(self->pw, PARAM_MAX_NUM_STEPS, 30);  
      bot_gtk_param_widget_set_int(self->pw, PARAM_MIN_NUM_STEPS, 0);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_STEP_SPEED, 0.8);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_STEP_HEIGHT, 0.1);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_NOM_FORWARD_STEP, 0.3);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_MAX_FORWARD_STEP, 0.5);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_NOM_STEP_WIDTH, 0.26);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_MU, 0.2);  
      bot_gtk_param_widget_set_enum(self->pw, PARAM_BEHAVIOR, BEHAVIOR_CRAWLING);
    }else if (mode == WALKING_BDI){
      std::cout << "Using preset mode: BDI\n";            
      bot_gtk_param_widget_set_int(self->pw, PARAM_MAX_NUM_STEPS, 10);  
      bot_gtk_param_widget_set_int(self->pw, PARAM_MIN_NUM_STEPS, 0);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_STEP_SPEED, 1.0);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_STEP_HEIGHT, 0.05);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_NOM_FORWARD_STEP, 0.1);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_MAX_FORWARD_STEP, 0.5);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_NOM_STEP_WIDTH, 0.26);  
      bot_gtk_param_widget_set_double(self->pw, PARAM_MU, 1.0);  
      bot_gtk_param_widget_set_enum(self->pw, PARAM_BEHAVIOR, BEHAVIOR_BDI_WALKING);
    }
  }  
  
  
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
  self->behavior = (behavior_t) bot_gtk_param_widget_get_enum(self->pw, PARAM_BEHAVIOR);
  self->goal_type = (walking_goal_type_t) bot_gtk_param_widget_get_enum(self->pw, PARAM_GOAL_TYPE);
  self->leading_foot = (leading_foot_t) bot_gtk_param_widget_get_enum(self->pw, PARAM_LEADING_FOOT);

  if(!strcmp(name, PARAM_GOAL_UPDATE)) {
    fprintf(stderr, "\nClicked Update Walking Goal\n");
    if (self->has_walking_msg) {
      publish_walking_goal(self, FALSE);
    }
  }else if(!strcmp(name, PARAM_GOAL_SEND)) {
    fprintf(stderr,"\nClicked WALKING_GOAL\n");
    //bot_viewer_request_pick (self->viewer, &(self->ehandler));
    activate(self);
  }
}

void publish_simple_nav(RendererWalking* self, double x, double y, double yaw) {
  double rpy[3];
  double quat[4];
  bot_quat_to_roll_pitch_yaw(self->robot_rot, rpy);
  rpy[2] += yaw;
  self->goal_yaw = rpy[2];
  bot_roll_pitch_yaw_to_quat(rpy, quat);
  self->goal_pos.x = self->robot_pos[0] + x * cos(rpy[2]) - y * sin(rpy[2]);
  self->goal_pos.y = self->robot_pos[1] + x * sin(rpy[2]) + y * cos(rpy[2]);
  bool old_follow_spline = self->follow_spline;
  self->follow_spline = FALSE;
  publish_walking_goal(self, TRUE);
  self->follow_spline = old_follow_spline;
}

void publish_walking_goal(RendererWalking* self, bool is_new) {
  drc_walking_goal_t walking_goal_msg;
  self->has_walking_msg = TRUE;
  double rpy[] = {0,0,self->goal_yaw};
  double quat_out[4];
  bot_roll_pitch_yaw_to_quat(rpy, quat_out); // its in w,x,y,z format
  walking_goal_msg.utime = self->robot_utime; //bot_timestamp_now();
  walking_goal_msg.max_num_steps = (int32_t) self->max_num_steps;
  walking_goal_msg.min_num_steps = (int32_t) self->min_num_steps;
  walking_goal_msg.goal_pos.translation.x = self->goal_pos.x;
  walking_goal_msg.goal_pos.translation.y = self->goal_pos.y;
  walking_goal_msg.goal_pos.translation.z = 0;
  walking_goal_msg.goal_pos.rotation.w = quat_out[0];
  walking_goal_msg.goal_pos.rotation.x = quat_out[1];
  walking_goal_msg.goal_pos.rotation.y = quat_out[2];
  walking_goal_msg.goal_pos.rotation.z = quat_out[3];
  walking_goal_msg.is_new_goal = is_new;
  walking_goal_msg.allow_optimization = self->allow_optimization;
  walking_goal_msg.step_speed = self->step_speed;
  walking_goal_msg.nom_step_width = self->nom_step_width;
  walking_goal_msg.nom_forward_step = self->nom_forward_step;
  walking_goal_msg.max_forward_step = self->max_forward_step;
  walking_goal_msg.step_height = self->step_height;
  walking_goal_msg.mu = self->mu;
  walking_goal_msg.follow_spline = self->follow_spline;
  walking_goal_msg.ignore_terrain = self->ignore_terrain;
  walking_goal_msg.behavior = self->behavior;
  walking_goal_msg.goal_type = self->goal_type;
  if (self->leading_foot == LEADING_FOOT_RIGHT) {
    walking_goal_msg.right_foot_lead = true;
  } else {
    walking_goal_msg.right_foot_lead = false;
  }
  std::string channel = (self->behavior == BEHAVIOR_CRAWLING) ? "CRAWLING_GOAL" : "WALKING_GOAL";
  fprintf(stderr, "Sending %s \n", channel.c_str());
  drc_walking_goal_t_publish(self->lc, channel.c_str(), &(walking_goal_msg));
  bot_viewer_set_status_bar_message(self->viewer, ("Sent " + channel).c_str());
}


static gboolean on_turn_left_clicked(GtkButton* button, void *user) {
  std::cout << "turn left" << std::endl;
  RendererWalking *self = (RendererWalking*) user;
  publish_simple_nav(self, 0, 0, M_PI / 2);
}
static gboolean on_go_forward_clicked(GtkButton* button, void *user) {
  std::cout << "go forward" << std::endl;
  RendererWalking *self = (RendererWalking*) user;
  publish_simple_nav(self, self->max_forward_step * self->max_num_steps, 0, 0);
}
static gboolean on_turn_right_clicked(GtkButton* button, void *user) {
  std::cout << "turn right" << std::endl;
  RendererWalking *self = (RendererWalking*) user;
  publish_simple_nav(self, 0, 0, -M_PI / 2);
}
static gboolean on_go_left_clicked(GtkButton* button, void *user) {
  std::cout << "go left" << std::endl;
  RendererWalking *self = (RendererWalking*) user;
  publish_simple_nav(self, 0, self->max_forward_step * self->max_num_steps, 0);
}
static gboolean on_go_backward_clicked(GtkButton* button, void *user) {
  std::cout << "go backward" << std::endl;
  RendererWalking *self = (RendererWalking*) user;
  publish_simple_nav(self, -self->max_forward_step * self->max_num_steps, 0, 0);
}
static gboolean on_go_right_clicked(GtkButton* button, void *user) {
  std::cout << "go right" << std::endl;
  RendererWalking *self = (RendererWalking*) user;
  publish_simple_nav(self, 0, -self->max_forward_step * self->max_num_steps, 0);
}

static void on_est_robot_state (const lcm_recv_buf_t * buf, const char *channel, 
                               const drc_robot_state_t *msg, void *user){
  RendererWalking *self = (RendererWalking*) user;
  
  self->robot_utime = msg->utime;
  self->robot_pos[0] = msg->pose.translation.x;
  self->robot_pos[1] = msg->pose.translation.y;
  self->robot_pos[2] = msg->pose.translation.z;
  self->robot_rot[0] = msg->pose.rotation.w;
  self->robot_rot[1] = msg->pose.rotation.x;
  self->robot_rot[2] = msg->pose.rotation.y;
  self->robot_rot[3] = msg->pose.rotation.z;
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
  self->max_num_steps = 10;
  self->min_num_steps = 0;
  self->step_speed = 1.0; // m/s
  self->step_height = 0.05; // m
  self->nom_forward_step = 0.10; // m
  self->max_forward_step = 0.5; // m
  self->nom_step_width = 0.26; // m
  self->behavior = BEHAVIOR_BDI_WALKING;
  self->walking_settings = WALKING_BDI;
  self->follow_spline = true;
  self->ignore_terrain = false;
  self->goal_type = GOAL_TYPE_CENTER;
  self->allow_optimization = false;
  self->mu = 1.0;
  self->leading_foot = LEADING_FOOT_RIGHT;
  self->robot_rot[0] = 1;
  self->robot_rot[1] = 0;
  self->robot_rot[2] = 0;
  self->robot_rot[3] = 0;
  
  self->perceptionData = new PerceptionData();
  self->perceptionData->mBotWrapper.reset(new maps::BotWrapper(lcm,param,frames));
  self->perceptionData->mViewClient.setBotWrapper(self->perceptionData->mBotWrapper);
  self->perceptionData->mViewClient.start();
  
  drc_robot_state_t_subscribe(self->lc,"EST_ROBOT_STATE",on_est_robot_state,self); 

  self->renderer.widget = gtk_alignment_new(0,0.5,1.0,0);

  GtkWidget *box;
  GtkWidget *arrow_table;
  // go_forward_button = gtk_button_new_with_label(PARAM_GO_FORWARD);

  box = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(self->renderer.widget), box);
  gtk_widget_show(box);

  arrow_table = gtk_table_new(1, 4, FALSE);
  gtk_box_pack_start(GTK_BOX(box), arrow_table, FALSE, TRUE, 0);
  gtk_widget_show(arrow_table);

  GtkWidget *go_forward_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_UP);
  GtkWidget *turn_left_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_UNDO);
  GtkWidget *turn_right_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_REDO);
  GtkWidget *go_left_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_BACK);
  GtkWidget *go_backward_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_DOWN);
  GtkWidget *go_right_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_GO_FORWARD);
  GtkWidget *nav_label = gtk_label_new("Quick Navigation:");

  gtk_table_attach(GTK_TABLE(arrow_table), nav_label, 0,1,0,2, (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), 0, 0);
  gtk_widget_show(nav_label);
  gtk_table_attach(GTK_TABLE(arrow_table), turn_left_button, 1, 2, 0, 1, (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), 0, 0);
  gtk_widget_show(turn_left_button);
  gtk_table_attach(GTK_TABLE(arrow_table), go_forward_button, 2, 3, 0, 1, (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), 0, 0);
  gtk_widget_show(go_forward_button);
  gtk_table_attach(GTK_TABLE(arrow_table), turn_right_button, 3, 4, 0, 1, (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), 0, 0);
  gtk_widget_show(turn_right_button);
  gtk_table_attach(GTK_TABLE(arrow_table), go_left_button, 1, 2, 1, 2, (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), 0, 0);
  gtk_widget_show(go_left_button);
  gtk_table_attach(GTK_TABLE(arrow_table), go_backward_button, 2, 3, 1, 2, (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), 0, 0);
  gtk_widget_show(go_backward_button);
  gtk_table_attach(GTK_TABLE(arrow_table), go_right_button, 3, 4, 1, 2, (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), (GtkAttachOptions)(GTK_FILL | GTK_SHRINK), 0, 0);
  gtk_widget_show(go_right_button);

  g_signal_connect(G_OBJECT(turn_left_button),
                   "clicked",
                   G_CALLBACK(on_turn_left_clicked),
                   self);
  g_signal_connect(G_OBJECT(go_forward_button),
                   "clicked",
                   G_CALLBACK(on_go_forward_clicked),
                   self);
  g_signal_connect(G_OBJECT(turn_right_button),
                   "clicked",
                   G_CALLBACK(on_turn_right_clicked),
                   self);
  g_signal_connect(G_OBJECT(go_left_button),
                   "clicked",
                   G_CALLBACK(on_go_left_clicked),
                   self);
  g_signal_connect(G_OBJECT(go_backward_button),
                   "clicked",
                   G_CALLBACK(on_go_backward_clicked),
                   self);
  g_signal_connect(G_OBJECT(go_right_button),
                   "clicked",
                   G_CALLBACK(on_go_right_clicked),
                   self);

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  gtk_box_pack_start(GTK_BOX(box), GTK_WIDGET(self->pw), FALSE, TRUE, 0);
  gtk_widget_show(GTK_WIDGET(self->pw));

  bot_gtk_param_widget_add_buttons(self->pw, PARAM_GOAL_SEND, NULL);
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_GOAL_UPDATE, NULL);
  bot_gtk_param_widget_add_enum(self->pw, PARAM_BEHAVIOR, BOT_GTK_PARAM_WIDGET_MENU, self->behavior, "Walking", BEHAVIOR_WALKING, "Crawling", BEHAVIOR_CRAWLING, "BDI Walking", BEHAVIOR_BDI_WALKING, "BDI Stepping", BEHAVIOR_BDI_STEPPING, NULL);
  bot_gtk_param_widget_add_enum(self->pw, PARAM_LEADING_FOOT, BOT_GTK_PARAM_WIDGET_MENU, self->leading_foot, "Right", LEADING_FOOT_RIGHT, "Left", LEADING_FOOT_LEFT, NULL);
  bot_gtk_param_widget_add_enum(self->pw, PARAM_GOAL_TYPE, BOT_GTK_PARAM_WIDGET_MENU, self->goal_type, "Bot center", GOAL_TYPE_CENTER, "Right foot", GOAL_TYPE_RIGHT_FOOT, "Left foot", GOAL_TYPE_LEFT_FOOT, NULL);
  bot_gtk_param_widget_add_int(self->pw, PARAM_MAX_NUM_STEPS, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 30, 1, self->max_num_steps);  
  bot_gtk_param_widget_add_int(self->pw, PARAM_MIN_NUM_STEPS, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 30, 1, self->min_num_steps);  
  bot_gtk_param_widget_add_double(self->pw, PARAM_STEP_SPEED, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.2, 5.0, 0.1, self->step_speed);
  bot_gtk_param_widget_add_double(self->pw, PARAM_STEP_HEIGHT, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.05, 0.5, 0.05, self->step_height);
  bot_gtk_param_widget_add_double(self->pw, PARAM_NOM_FORWARD_STEP, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.05, 1.0, 0.05, self->nom_forward_step);
  bot_gtk_param_widget_add_double(self->pw, PARAM_MAX_FORWARD_STEP, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.05, 1.0, 0.05, self->max_forward_step);
  bot_gtk_param_widget_add_double(self->pw, PARAM_NOM_STEP_WIDTH, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.22, 0.4, 0.01, self->nom_step_width);
  bot_gtk_param_widget_add_double(self->pw, PARAM_MU, BOT_GTK_PARAM_WIDGET_SPINBOX, 0.0, 1.5, 0.05, self->mu);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_FOLLOW_SPLINE, 0, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_IGNORE_TERRAIN, 0, NULL);
  bot_gtk_param_widget_set_bool(self->pw, PARAM_FOLLOW_SPLINE, self->follow_spline);
  bot_gtk_param_widget_set_bool(self->pw, PARAM_IGNORE_TERRAIN, self->ignore_terrain);
  
  bot_gtk_param_widget_add_enum(self->pw, WALKING_MODE, BOT_GTK_PARAM_WIDGET_MENU, self->walking_settings, 
                                "BDI Walking", WALKING_BDI,
                                "Typical, VRC", WALKING_TYPICAL,
                                "Mud, VRC", WALKING_MUD,
                                "Crawling", WALKING_CRAWLING,
                                "Turn Crawl", WALKING_TURN_CRAWLING ,
                                NULL);


  // bot_gtk_param_widget_add_buttons(self->pw, PARAM_GO_FORWARD, NULL);

  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  // self->renderer.widget = GTK_WIDGET(self->pw);

  self->active = false;

  return &self->renderer;
}

void setup_renderer_walking(BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param,
    BotFrames * frames)
{
  bot_viewer_add_renderer_on_side(viewer, renderer_walking_new(viewer, render_priority, lcm, param, frames),
      render_priority , 0);
}
