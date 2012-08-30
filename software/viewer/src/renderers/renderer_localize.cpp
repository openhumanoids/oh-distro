// Renderer for a point-and-click message publisher
// used to send a message to relocalize a robot
// this was orginally part of envoy/renderers
// mfallon aug2011
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/bot_core.h>

#define RENDERER_NAME "Localize"
#define PARAM_GOAL "Goal [G]"
#define PARAM_REINITIALIZE "Reinit [R]"
#define PARAM_NEW_MAP "New Map"
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

typedef struct _RendererNavigation {
  BotRenderer renderer;
  BotEventHandler ehandler;
  BotViewer *viewer;
  lcm_t *lc;

  BotGtkParamWidget *pw;

  int dragging;
  int active; //1 = relocalize, 2 = set person location
  int last_active; //1 = relocalize, 2 = set person location
  point2d_t drag_start_local;
  point2d_t drag_finish_local;

  point2d_t click_pos;
  double theta;
  double goal_std;

  int64_t max_draw_utime;

}RendererNavigation;

static void
_draw (BotViewer *viewer, BotRenderer *renderer)
{
  RendererNavigation *self = (RendererNavigation*) renderer;
  int64_t now = bot_timestamp_now();

  if(!self->dragging && now > self->max_draw_utime)
    return;

  glColor3f(0, 1, 0);
  glPushMatrix();
  glTranslatef(self->click_pos.x, self->click_pos.y, 0);

  bot_gl_draw_circle(self->goal_std);

  glBegin(GL_LINE_STRIP);
  glVertex2f(0.0,0.0);

  glVertex2f(self->goal_std*cos(self->theta),self->goal_std*sin(self->theta));
  glEnd();

  glPopMatrix();

}

static void
recompute_2d_goal_pose(RendererNavigation *self)
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
  RendererNavigation *self = (RendererNavigation*) ehandler->user;

  //fprintf(stderr, "Active: %d | Mouse Press : %f,%f\n",self->active, ray_start[0], ray_start[1]);

  self->dragging = 0;

  if(ehandler->picking==0){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  if(self->active==0){
    fprintf(stderr, "Not Active\n");
    return 0;
  }

  if(event->button != 1){
    fprintf(stderr,"Wrong Button\n");
    return 0;
  }

  point2d_t click_pt_local;
  if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start),
      POINT3D(ray_dir), 0, &click_pt_local)) {
    bot_viewer_request_redraw(self->viewer);
    self->active = 0;
    return 0;
  }

  self->dragging = 1;

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
  RendererNavigation *self = (RendererNavigation*) ehandler->user;

  if (self->dragging) {
    self->dragging = 0;
  }
  if (self->active != 0) {
    // check drag points and publish

    printf("x,y,t: %f %f %f.    std: %f\n",self->click_pos.x
        ,self->click_pos.y,self->theta,self->goal_std);

    drc_localize_reinitialize_cmd_t msg;
    msg.utime = bot_timestamp_now();
    msg.mean[0] = self->click_pos.x;
    msg.mean[1] = self->click_pos.y;
    msg.mean[2] = self->theta;

    double v = self->goal_std * self->goal_std;
    msg.variance[0] = v;
    msg.variance[1] = v;
    msg.variance[2] = VARIANCE_THETA;

    fprintf(stderr,"Localizer Button Released => Activate Value : %d\n", self->active);
    if(self->active == 1){
      fprintf(stderr, "Sending LOCALIZE_REINITIALIZE\n");
      drc_localize_reinitialize_cmd_t_publish(self->lc, "LOCALIZE_REINITIALIZE", &msg);
    }else if (self->active ==2){

      drc_nav_goal_t msg;
      msg.utime = bot_timestamp_now();
      msg.robot_name = "wheeled_atlas"; // this should be set from robot state message

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
      fprintf(stderr, "Sending NAV_GOAL\n");
      drc_nav_goal_t_publish(self->lc, "NAV_GOAL", &msg);

    }
    bot_viewer_set_status_bar_message(self->viewer, "sent msg");
    self->last_active = self->active;
    self->active = 0;

    ehandler->picking = 0;
    return 1;
  }
  else
    ehandler->picking = 0;

  return 0;
}

static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,
    const double ray_start[3], const double ray_dir[3],
    const GdkEventMotion *event)
{
  RendererNavigation *self = (RendererNavigation*) ehandler->user;

  if(!self->dragging || self->active==0)
    return 0;

  point2d_t drag_pt_local;
  if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start),
      POINT3D(ray_dir), 0, &drag_pt_local)) {
    return 0;
  }
  self->drag_finish_local = drag_pt_local;
  recompute_2d_goal_pose(self);

  bot_viewer_request_redraw(self->viewer);
  return 1;
}

void activate(RendererNavigation *self, int type)
{
  self->active = type;
}

static int key_press (BotViewer *viewer, BotEventHandler *ehandler, 
    const GdkEventKey *event)
{
  RendererNavigation *self = (RendererNavigation*) ehandler->user;

  if ((event->keyval == 'r' || event->keyval == 'R') && self->active==0) {
    printf("\n[R]einit key registered\n");
    activate(self,1);
    bot_viewer_request_pick (viewer, ehandler);
  }else if ((event->keyval == 'g' || event->keyval == 'G') && self->active==0) {
    printf("\n[G]oal key registered\n");
    activate(self,2);
    bot_viewer_request_pick (viewer, ehandler);
  } else if(event->keyval == GDK_Escape) {
    //self->active = 0;
    //ehandler->picking = 0;
    //bot_viewer_set_status_bar_message(self->viewer, "");
  }

  return 0;
}


// Send a message to start a new map
// this is really lasy as it reuses the reinitialse message
// i did this to avoid inventing another
static void send_new_map (RendererNavigation *self){
  BotViewHandler *vhandler = self->viewer->view_handler;  

  drc_localize_reinitialize_cmd_t msg;
  msg.utime = bot_timestamp_now();
  msg.mean[0] = -99999;
  msg.mean[1] = -99999;
  msg.mean[2] = -99999;
  msg.variance[0] = -99999;
  msg.variance[1] = -99999;
  msg.variance[2] = -99999;
  fprintf(stderr,"\nSending LOCALIZE_NEW_MAP message\n");//, self->active);
  drc_localize_reinitialize_cmd_t_publish(self->lc, "LOCALIZE_NEW_MAP", &msg);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererNavigation *self = (RendererNavigation*) user;
  if(!strcmp(name, PARAM_REINITIALIZE)) {
    fprintf(stderr,"\nClicked REINIT\n");
    bot_viewer_request_pick (self->viewer, &(self->ehandler));
    activate(self, 1);
  }else if(!strcmp(name, PARAM_GOAL)) {
    fprintf(stderr,"\nClicked NAV_GOAL\n");
    bot_viewer_request_pick (self->viewer, &(self->ehandler));
    activate(self, 2);
  }else if (! strcmp (name, PARAM_NEW_MAP)) {
    send_new_map(self);
  }
}

static void
_free (BotRenderer *renderer)
{
  free (renderer);
}

BotRenderer *renderer_localize_new (BotViewer *viewer, int render_priority, lcm_t *lcm)
{
  RendererNavigation *self = (RendererNavigation*) calloc (1, sizeof (RendererNavigation));
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

  self->lc = lcm; //globals_get_lcm_full(NULL,1);

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_GOAL, NULL);
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_REINITIALIZE, NULL);
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_NEW_MAP, NULL);

  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  self->renderer.widget = GTK_WIDGET(self->pw);

  self->active = 0;

  return &self->renderer;
}

void setup_renderer_localize(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
  bot_viewer_add_renderer(viewer, renderer_localize_new(viewer, render_priority, lcm),
      render_priority);
}
