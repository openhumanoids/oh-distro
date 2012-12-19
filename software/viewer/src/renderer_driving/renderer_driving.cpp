// Renderer for a point-and-click message publisher
// used to send a message to relocalize a robot
// this was orginally part of envoy/renderers
// mfallon aug2011
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>

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

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>
//#include <visualization/renderer_localize.h>

#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/perception_pointing_vector_t.h>


#define RENDERER_NAME "Driving"
#define PARAM_GOAL_SEND "[G]oal (timed)"
#define PARAM_GOAL_TIMEOUT "Goal Lifespan"
#define PARAM_GOAL_SEND_LEFT_HAND "[L]eft Hand Goal"
#define PARAM_VISUAL_GOAL_TYPE "VisGoalType"
#define PARAM_VISUAL_GOAL      "Visual Goal"
#define PARAM_NEW_MAP "New Map"
#define PARAM_NEW_OCTOMAP "New OctoMap"
#define PARAM_HEIGHTMAP_RES "Heightmap Res"
#define PARAM_UPDATE_HEIGHTMAP "Update Heightmap"


typedef enum _heightmap_res_t {
  HEIGHTMAP_RES_HIGH, HEIGHTMAP_RES_LOW,
} heightmap_res_t;

typedef enum _visual_goal_type_t {
  VISUAL_GOAL_GLOBAL, VISUAL_GOAL_RELATIVE
} visual_goal_type_t;


// Controlling Spinning Lidar:
#define PARAM_LIDAR_RATE "Lidar Rate"
#define PARAM_LIDAR_RATE_SEND "Send Rate"

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

using namespace std;


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

Eigen::Quaterniond euler_to_quat(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}

void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}


// modulu - similar to matlab's mod()
// result is always possitive. not similar to fmod()
// Mod(-3,4)= 1   fmod(-3,4)= -3
double Mod(double x, double y)
{
    if (0 == y)
        return x;

    return x - y * floor(x/y);
}

// wrap [rad] angle to [-PI..PI)
double WrapPosNegPI(double fAng)
{
    return Mod(fAng + M_PI, M_PI*2) - M_PI;
}




////////////////////////////// END OF CODE COPIED IN FROM COMMON_UTILS

typedef struct _RendererDriving {
  BotRenderer renderer;
  BotEventHandler ehandler;

  BotParam *bot_param;
  BotFrames *bot_frames;
  bot::frames* bot_frames_cpp;    
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
  double goal_timeout; // no. seconds before this goal expires
  heightmap_res_t heightmap_res;

  int64_t max_draw_utime;
  double circle_color[3];
  
  // Most recent robot position, rotation and utime
  int64_t robot_utime;
  Eigen::Isometry3d robot_pose;
  double robot_yaw;

  double min_turn_radius;
  double wheel_separation; // left/right distance between the wheels
  Eigen::Isometry3d center_arc_left;
  Eigen::Isometry3d center_arc_right;
  double max_velocity;
  
  //

  
  
  visual_goal_type_t visual_goal_type;
  // Local [use when visual navigation works]
  Eigen::Vector4d local_pointpose_from;
  Eigen::Vector4d local_pointpose_to;
  // Body [use when visual navigation fails]
  Eigen::Vector4d body_pointpose_from;
  Eigen::Vector4d body_pointpose_to;
  
  
  // Frequency of rotating lidar in hz:
  double lidar_rate;

}RendererDriving;

static void
_draw (BotViewer *viewer, BotRenderer *renderer)
{
  RendererDriving *self = (RendererDriving*) renderer;
  int64_t now = bot_timestamp_now();

  
  glLineWidth(2.0);
  glPushMatrix();
  if (self->visual_goal_type == VISUAL_GOAL_GLOBAL){
    glColor3f(0,1,1); //cyan
  }else{
    glColor3f(0.65,0.16,0.16); //brown
  }
  glBegin(GL_LINE_STRIP);
  glVertex3f(self->local_pointpose_from[0], self->local_pointpose_from[1],
              self->local_pointpose_from[2] );
  glVertex3f(self->local_pointpose_to[0], self->local_pointpose_to[1],
              self->local_pointpose_to[2] );

  glEnd();
  glPopMatrix();
  
  
  
  
  // select xy (theta) point
  // convert to local space
  // determine if within feasable region
  // draw
  
  // Circles representing the left and right wheels at the limits:
  glLineWidth(2.0);
  glPushMatrix();
  glTranslatef(self->center_arc_left.translation().x() , self->center_arc_left.translation().y() , self->center_arc_left.translation().z());
  glColor3f(0,0.5,1);
  bot_gl_draw_circle( self->min_turn_radius - self->wheel_separation/2.0 ); // inner radius
  glColor3f(1.0,0.1,0);
  bot_gl_draw_circle( self->min_turn_radius );
  glPopMatrix();
  glPushMatrix();
  glTranslatef(self->center_arc_right.translation().x() , self->center_arc_right.translation().y() , self->center_arc_right.translation().z());
  glColor3f(0,0.5,1);
  bot_gl_draw_circle( self->min_turn_radius - self->wheel_separation/2.0 ); // inner radius
  glColor3f(1.0,0.1,0);
  bot_gl_draw_circle( self->min_turn_radius );
  glPopMatrix();

  // Maximum permissable distance arc:
  double max_permissable_travel_distance =  self->goal_timeout * self->max_velocity;
  //cout << "max_permissable_travel_distance: " << max_permissable_travel_distance << "\n";
  glColor3f(1.0,0.1,0);
  glPushMatrix();
  glTranslatef(self->robot_pose.translation().x() , self->robot_pose.translation().y() , self->robot_pose.translation().z());
  glRotatef(self->robot_yaw*180/M_PI , 0, 0, 1.0); // rotate on z-axis
  // Line straight out in front:
  glBegin(GL_LINE_STRIP);
  glVertex2f(0.0,0.0);
  glVertex2f(max_permissable_travel_distance, 0);
  glEnd();
  for (int i=-1; i<=1 ; i=i+2){
    glBegin(GL_LINE_STRIP);
    glVertex2f(max_permissable_travel_distance, 0);
    for (double count = 0.001; count < 1 ; count =count+0.01){
      double r = 1/count;
      double circ = 2*M_PI*r;  
      double fraction =  max_permissable_travel_distance/circ;
      double boundary_x = r*sin( fraction *2*M_PI);
      double boundary_y = r*cos( fraction *2*M_PI);

      if (sqrt(boundary_x*boundary_x +  (boundary_y)*(boundary_y))  <  self->min_turn_radius ){
        // cout << r << " \n";
        break; 
      }
      
      if (i > 0){
        glVertex2f(boundary_x, boundary_y - r);
      }else{
        glVertex2f(boundary_x, r -boundary_y );
      }
    }
  
    double r = self->min_turn_radius;
    double circ = 2*M_PI*r;  
    double fraction =  max_permissable_travel_distance/circ;
    double boundary_x = r*sin( fraction *2*M_PI);
    double boundary_y = r*cos( fraction *2*M_PI);

    if (i > 0){
      glVertex2f(boundary_x, boundary_y - r);
    }else{
      glVertex2f(boundary_x, r -boundary_y );
    }
  
    glEnd();
  }
  glPopMatrix();
  

  // Determine if end goal is within the permissable region:
  // NB: this code mixes the relative openGL axes and the relative northings|eastings  axes
  // 1. Convert the point into a relative angle and range
  double dx = self->click_pos.x - self->robot_pose.translation().x(); 
  double dy = self->click_pos.y - self->robot_pose.translation().y();
  //cout << dx << " and " << dy <<" top\n";
  double click_rel_heading =  atan2(dy, dx)   - self->robot_yaw;
  click_rel_heading  = WrapPosNegPI (click_rel_heading );   // click rel heading: + to left | - to right. forward 0 behind +/ pi
  double click_pos_rel_range = sqrt(dx*dx + dy*dy);
   // Only used below for renderering:
  double gl_click_pos_rel_x =  click_pos_rel_range * cos(click_rel_heading);
  double gl_click_pos_rel_y =  click_pos_rel_range * sin(click_rel_heading);
  ////////////////////////////////////
  click_rel_heading = click_rel_heading + M_PI/2; // convert to theta=0 at eastings convention
  
  // 2. Determine the radius and path length
  double xyframe_rel_x = click_pos_rel_range * cos(click_rel_heading);
  double xyframe_rel_y = click_pos_rel_range * sin(click_rel_heading);  
  double path_R =100;
  double path_MaxTheta = 0.8;
  double path_MinTheta =0;
  double path_center=0;
  double path_Angle = 0;
  if (click_rel_heading >=  M_PI/2){ //LHS of robot
      double alpha = M_PI/2 - click_rel_heading;
      double h = xyframe_rel_y/(2* sin(alpha));
      path_R = abs( h/sin( click_rel_heading) );
      path_center = path_R;
      path_Angle = abs(alpha*2);
      path_MaxTheta = path_Angle;
      path_MinTheta =0;
  }else{ // RHS of robot
      double alpha = M_PI/2 - click_rel_heading;
      double h = xyframe_rel_y/(2* sin(alpha));
      path_R = abs( h/sin( click_rel_heading) );
      path_center = -path_R;
      path_Angle = abs(alpha*2);
      path_MaxTheta =M_PI ;
      path_MinTheta =M_PI  - path_Angle;
  }
  double frac = path_Angle / (2*M_PI);
  double proposed_distance = frac *2*M_PI * path_R;
  //cout << "frac: " << frac << "\n";
  //cout << "proposed_distance: " << proposed_distance << "\n";
  
  // the end goal in openGL coordinates:
  glColor3f(0, 1,0.5 );
  glPushMatrix();
  glPointSize(5.0f);
  glBegin(GL_POINTS);
  glVertex2f(self->click_pos.x, self->click_pos.y);
  glEnd();
  glPopMatrix();  
  // the end goal in the robot frame:
  glColor3f(0.0, 0.4,0.05 );
  glPushMatrix();
  glTranslatef(self->robot_pose.translation().x() , self->robot_pose.translation().y() , self->robot_pose.translation().z());
  glRotatef(self->robot_yaw*180/M_PI , 0, 0, 1.0); // rotate on z-axis
  glPointSize(5.0f);
  glBegin(GL_POINTS);
  glVertex2f(gl_click_pos_rel_x, gl_click_pos_rel_y);
  glEnd();
  glPopMatrix();      

  // if in permisable region - draw it.
  if ((proposed_distance < max_permissable_travel_distance) && 
     (path_R > self->min_turn_radius )) {
  
    glPushMatrix();
    glTranslatef(self->robot_pose.translation().x() , self->robot_pose.translation().y() , self->robot_pose.translation().z());
    glRotatef(self->robot_yaw*180/M_PI , 0, 0, 1.0); // rotate on z-axis
    // OpenGL is x forward, y left
    double path_x, path_y;
    glBegin(GL_LINE_STRIP);
    for (double i= path_MinTheta;i< path_MaxTheta ; i=i+0.025){
      path_x = path_R*sin(i);
      path_y = - path_R*cos(i);
      glVertex2f(path_x, path_y + path_center); 
    }
    path_x = path_R*sin(path_MaxTheta);
    path_y = - path_R*cos(path_MaxTheta);
    glVertex2f(path_x, path_y + path_center);
    glEnd();
    glPopMatrix();
    bot_viewer_set_status_bar_message(self->viewer, "Valid Goal, click ok to send it");
  }else{
     bot_viewer_set_status_bar_message(self->viewer, "Invalid Goal: too far or too tight");
  }
  
/*  
 // Old stuff from Navigator circle
  if(!self->dragging && now > self->max_draw_utime)
    return;

  //glColor3f(0, 1, 0);
  glColor3f(self->circle_color[0], self->circle_color[1], self->circle_color[2]);
  glPushMatrix();
  glTranslatef(self->click_pos.x, self->click_pos.y, 0);

  bot_gl_draw_circle(self->goal_std);

  glBegin(GL_LINE_STRIP);
  glVertex2f(0.0,0.0);

  glVertex2f(self->goal_std*cos(self->theta),self->goal_std*sin(self->theta));
  glEnd();

  glPopMatrix();
  
  */


}

static void
recompute_2d_goal_pose(RendererDriving *self)
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
  RendererDriving *self = (RendererDriving*) ehandler->user;

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
  RendererDriving *self = (RendererDriving*) ehandler->user;

  if (self->dragging) {
    self->dragging = 0;
  }
  if (self->active != 0) {
    // check drag points and publish

    printf("x,y,t: %f %f %f.    std: %f\n",self->click_pos.x
        ,self->click_pos.y,self->theta,self->goal_std);

    fprintf(stderr,"Localizer Button Released => Activate Value : %d\n", self->active);
    if(self->active == 1){
      drc_localize_reinitialize_cmd_t msg;
      msg.utime = self->robot_utime; //bot_timestamp_now();
      msg.mean[0] = self->click_pos.x;
      msg.mean[1] = self->click_pos.y;
      msg.mean[2] = self->theta;

      double v = self->goal_std * self->goal_std;
      msg.variance[0] = v;
      msg.variance[1] = v;
      msg.variance[2] = VARIANCE_THETA;
      fprintf(stderr, "Sending LOCALIZE_REINITIALIZE\n");
      drc_localize_reinitialize_cmd_t_publish(self->lc, "LOCALIZE_REINITIALIZE", &msg);
      bot_viewer_set_status_bar_message(self->viewer, "Sent LOCALIZE_REINITIALIZE");

      self->circle_color[0] = 1;
      self->circle_color[1] = 0;
      self->circle_color[2] = 0;
    }else if (self->active ==2){
      drc_nav_goal_timed_t msg;
      msg.utime = self->robot_utime; //bot_timestamp_now();
      msg.timeout = (int64_t) 1E6*self->goal_timeout;
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
      fprintf(stderr, "Sending NAV_GOAL_TIMED\n");
      drc_nav_goal_timed_t_publish(self->lc, "NAV_GOAL_TIMED", &msg);
      bot_viewer_set_status_bar_message(self->viewer, "Sent NAV_GOAL_TIMED");
    }else if (self->active ==3){
      drc_nav_goal_t msg;
      msg.utime = self->robot_utime; // bot_timestamp_now();
      msg.robot_name = "wheeled_atlas"; // this should be set from robot state message

      msg.goal_pos.translation.x = self->click_pos.x;
      msg.goal_pos.translation.y = self->click_pos.y;
      msg.goal_pos.translation.z = 2; // hard coded for now
      double rpy[] = {0,0,self->theta};
      double quat_out[4];
      bot_roll_pitch_yaw_to_quat(rpy, quat_out); // its in w,x,y,z format
      msg.goal_pos.rotation.w = quat_out[0];
      msg.goal_pos.rotation.x = quat_out[1];
      msg.goal_pos.rotation.y = quat_out[2];
      msg.goal_pos.rotation.z = quat_out[3];
      fprintf(stderr, "Sending NAV_GOAL_LEFT_HAND\n");
      drc_nav_goal_t_publish(self->lc, "NAV_GOAL_LEFT_HAND", &msg);
      bot_viewer_set_status_bar_message(self->viewer, "Sent NAV_GOAL_LEFT_HAND");
    }
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
  RendererDriving *self = (RendererDriving*) ehandler->user;

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

void activate(RendererDriving *self, int type)
{
  self->active = type;

  self->goal_std=0;
  if (self->active ==1){
    self->circle_color[0] = 1;
    self->circle_color[1] = 0;
    self->circle_color[2] = 0;
  }else if (self->active ==2){
    self->circle_color[0] = 0;
    self->circle_color[1] = 1;
    self->circle_color[2] = 0;
  }else if (self->active ==3){
    self->circle_color[0] = 0;
    self->circle_color[1] = 0;
    self->circle_color[2] = 1;
  }
}

static int key_press (BotViewer *viewer, BotEventHandler *ehandler, 
    const GdkEventKey *event)
{
  RendererDriving *self = (RendererDriving*) ehandler->user;
  self->goal_timeout = bot_gtk_param_widget_get_double(self->pw, PARAM_GOAL_TIMEOUT);
  self->lidar_rate = bot_gtk_param_widget_get_double(self->pw, PARAM_LIDAR_RATE);

  if ((event->keyval == 'r' || event->keyval == 'R') && self->active==0) {
    printf("\n[R]einit key registered\n");
    activate(self,1);
    bot_viewer_request_pick (viewer, ehandler);
  }else if ((event->keyval == 'g' || event->keyval == 'G') && self->active==0) {
    printf("\n[G]oal (timed) key registered\n");
    activate(self,2);
    bot_viewer_request_pick (viewer, ehandler);
  }else if ((event->keyval == 'l' || event->keyval == 'L') && self->active==0) {
    printf("\n[L]eft arm key registered\n");
    activate(self,3);
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
static void send_new_map (RendererDriving *self){
  BotViewHandler *vhandler = self->viewer->view_handler;

  drc_localize_reinitialize_cmd_t msg;
  msg.utime = self->robot_utime;//bot_timestamp_now();
  msg.mean[0] = -99999;
  msg.mean[1] = -99999;
  msg.mean[2] = -99999;
  msg.variance[0] = -99999;
  msg.variance[1] = -99999;
  msg.variance[2] = -99999;
  fprintf(stderr,"\nSending LOCALIZE_NEW_MAP message\n");//, self->active);
  drc_localize_reinitialize_cmd_t_publish(self->lc, "LOCALIZE_NEW_MAP", &msg);
  bot_viewer_set_status_bar_message(self->viewer, "Sent LOCALIZE_NEW_MAP");
}


static void send_new_lidar_rate (RendererDriving *self){
  BotViewHandler *vhandler = self->viewer->view_handler;
  // This is a direct command of the rotation rate... 

  drc_twist_timed_t msg;
  msg.utime = self->robot_utime;//bot_timestamp_now();
  msg.angular_velocity.x = self->lidar_rate;
  msg.angular_velocity.y = 0.0;
  msg.angular_velocity.z = 0.0;
  msg.linear_velocity.x = 0.0;
  msg.linear_velocity.y = 0.0;
  msg.linear_velocity.z = 0.0;

  fprintf(stderr,"\nSending ROTATING_SCAN_RATE_CMD message %f\n",self->lidar_rate);//, self->active);
  drc_twist_timed_t_publish(self->lc, "ROTATING_SCAN_RATE_CMD", &msg);
  bot_viewer_set_status_bar_message(self->viewer, "Sent ROTATING_SCAN_RATE_CMD [%f Hz]",self->lidar_rate);
}


// Send a message to start a new OctoMap 
static void send_new_octomap (RendererDriving *self){
  BotViewHandler *vhandler = self->viewer->view_handler;

  drc_map_params_t msgout;
  msgout.utime = self->robot_utime;
  msgout.message_id = 0;
  msgout.map_id = -1;
  msgout.resolution = 0.02;
  msgout.dimensions[0] = 40;//10;
  msgout.dimensions[1] = 40;//10;
  msgout.dimensions[2] = 40;//10;
  
  msgout.transform_to_local.translation.x = self->robot_pose.translation().x();
  msgout.transform_to_local.translation.y = self->robot_pose.translation().y();
  msgout.transform_to_local.translation.z = self->robot_pose.translation().z();
  msgout.transform_to_local.rotation.x = 0;
  msgout.transform_to_local.rotation.y = 0;
  msgout.transform_to_local.rotation.z = 0;
  msgout.transform_to_local.rotation.w = 1; // to keep world aligned  

  drc_map_params_t_publish(self->lc,"MAP_CREATE",&msgout);
  bot_viewer_set_status_bar_message(self->viewer, "Sent MAP_CREATE");
}

static void update_heightmap (RendererDriving *self) {
  drc_heightmap_params_t msg;
  msg.utime = self->robot_utime;
  switch (self->heightmap_res) {
  case HEIGHTMAP_RES_HIGH:
    msg.resolution = 0.1; break;
  case HEIGHTMAP_RES_LOW:
    msg.resolution = 0.5; break;
  default:
    msg.resolution = 0.1; break;
  }
  drc_heightmap_params_t_publish(self->lc,"HEIGHTMAP_PARAMS", &msg);
  bot_viewer_set_status_bar_message(self->viewer, "Sent HEIGHTMAP_PARAMS");
}

static void send_seek_goal_visual (RendererDriving *self) {
  
  drc_seek_goal_timed_t msgout;
  msgout.utime = self->robot_utime; //bot_timestamp_now();
  msgout.timeout = (int64_t) 1E6*self->goal_timeout; //self->goal_timeout;
  msgout.robot_name = "wheeled_atlas"; // this should be set from robot state message
  if (self->visual_goal_type == VISUAL_GOAL_GLOBAL){
    msgout.type = DRC_SEEK_GOAL_TIMED_T_VISUAL_GLOBAL;
    fprintf(stderr, "Sending SEEK_GOAL (Visual, Global)\n");
    drc_seek_goal_timed_t_publish(self->lc, "SEEK_GOAL", &msgout); 
    bot_viewer_set_status_bar_message(self->viewer, "Sent SEEK_GOAL (Visual, Global) ");
  }else if (self->visual_goal_type == VISUAL_GOAL_RELATIVE){
    msgout.type = DRC_SEEK_GOAL_TIMED_T_VISUAL_RELATIVE;
    fprintf(stderr, "Sending SEEK_GOAL (Visual, Relative)\n");
    drc_seek_goal_timed_t_publish(self->lc, "SEEK_GOAL", &msgout);       
    bot_viewer_set_status_bar_message(self->viewer, "Sent SEEK_GOAL (Visual, Relative)");
  }  

  /*
  // Set goal:
  drc_nav_goal_timed_t msgout;
  msgout.utime = self->robot_utime; //bot_timestamp_now();
  msgout.timeout = (int64_t) 1E6*self->goal_timeout; //self->goal_timeout;
  msgout.robot_name = "wheeled_atlas"; // this should be set from robot state message
  msgout.goal_pos.translation.z = 0;
  msgout.goal_pos.rotation.w = 1; // Null heading
  msgout.goal_pos.rotation.x = 0;
  msgout.goal_pos.rotation.y = 0;
  msgout.goal_pos.rotation.z = 0;
  if (self->visual_goal_type == VISUAL_GOAL_GLOBAL){
    msgout.goal_pos.translation.x = self->local_pointpose_to[0];
    msgout.goal_pos.translation.y = self->local_pointpose_to[1];
    fprintf(stderr, "Sending NAV_GOAL_TIMED\n");
    drc_nav_goal_timed_t_publish(self->lc, "NAV_GOAL_TIMED", &msgout); 
    bot_viewer_set_status_bar_message(self->viewer, "Sent NAV_GOAL_TIMED (Visual) ");
  }else if (self->visual_goal_type == VISUAL_GOAL_RELATIVE){
    msgout.goal_pos.translation.x = self->body_pointpose_to[0];
    msgout.goal_pos.translation.y = self->body_pointpose_to[1];
    fprintf(stderr, "Sending RELATIVE_NAV_GOAL_TIMED\n");
    drc_nav_goal_timed_t_publish(self->lc, "RELATIVE_NAV_GOAL_TIMED", &msgout);       
    bot_viewer_set_status_bar_message(self->viewer, "Sent RELATIVE_NAV_GOAL_TIMED (Visual) ");
  }
  */
}


static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererDriving *self = (RendererDriving*) user;
  self->goal_timeout = bot_gtk_param_widget_get_double(self->pw, PARAM_GOAL_TIMEOUT);
  self->lidar_rate = bot_gtk_param_widget_get_double(self->pw, PARAM_LIDAR_RATE);
  self->heightmap_res =(heightmap_res_t)  bot_gtk_param_widget_get_enum(self->pw, PARAM_HEIGHTMAP_RES);
  self->visual_goal_type =(visual_goal_type_t)  bot_gtk_param_widget_get_enum(self->pw, PARAM_VISUAL_GOAL_TYPE);
  
  if(!strcmp(name, PARAM_VISUAL_GOAL)) {
    send_seek_goal_visual(self);
  }else if(!strcmp(name, PARAM_GOAL_SEND)) {
    fprintf(stderr,"\nClicked NAV_GOAL_TIMED\n");
    bot_viewer_request_pick (self->viewer, &(self->ehandler));
    activate(self, 2);
  }else if(!strcmp(name, PARAM_GOAL_SEND_LEFT_HAND)) {
    fprintf(stderr,"\nClicked NAV_GOAL_LEFT_HAND\n");
    bot_viewer_request_pick (self->viewer, &(self->ehandler));
    activate(self, 3);
  }else if(!strcmp(name, PARAM_LIDAR_RATE_SEND)) {
    fprintf(stderr,"\nClicked LIDAR_RATE_SEND\n");
    bot_viewer_request_pick (self->viewer, &(self->ehandler));
    send_new_lidar_rate(self);
  }else if (! strcmp (name, PARAM_NEW_MAP)) {
    send_new_map(self);
  }else if (! strcmp (name, PARAM_NEW_OCTOMAP)) {
    send_new_octomap(self);
  }else if (! strcmp (name, PARAM_UPDATE_HEIGHTMAP)) {
    update_heightmap(self);
  }
  
    bot_viewer_request_redraw(self->viewer);

}

static void on_est_robot_state (const lcm_recv_buf_t * buf, const char *channel, 
                               const drc_robot_state_t *msg, void *user){
  RendererDriving *self = (RendererDriving*) user;
  
  self->robot_utime =msg->utime;
  
  // Remove the pose roll and pitch:
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->origin_position.rotation.w, msg->origin_position.rotation.x, 
                                               msg->origin_position.rotation.y, msg->origin_position.rotation.z);
  double ypr[3];
  quat_to_euler(quat, ypr[0], ypr[1], ypr[2]);
  ypr[1] =0; ypr[2] =0;
  Eigen::Quaterniond quat_yaw_only = euler_to_quat(ypr[0], ypr[1], ypr[2]);
  self->robot_pose.setIdentity();
  self->robot_pose.translation()  << msg->origin_position.translation.x, msg->origin_position.translation.y, msg->origin_position.translation.z;
  self->robot_pose.rotate(quat_yaw_only);    
  self->robot_yaw = ypr[0]; // for convenece keep the yaw

  // Determine the left and right max turning circles:
  Eigen::Isometry3d offset;
  offset.setIdentity();
  offset.translation()  << 0, self->min_turn_radius,0;
  self->center_arc_left = self->robot_pose * offset;
  offset.translation()  << 0,- (self->min_turn_radius) ,0;
  self->center_arc_right = self->robot_pose * offset;
  bot_viewer_request_redraw(self->viewer);
}


static void on_pointing_vector(const lcm_recv_buf_t * buf, const char *channel, 
                               const perception_pointing_vector_t *msg, void *user){
  RendererDriving *self = (RendererDriving*) user;
  cout << "got pointing vector\n";
  
  // Convert vector into point very far away
  double scale = 100.0;
  double goal_pos[]={scale*msg->vec[0], scale*msg->vec[1],scale* msg->vec[2]};
  
  int status_local,status_body;
  Eigen::Isometry3d cam_to_local;
  Eigen::Isometry3d cam_to_body;
  status_local = self->bot_frames_cpp->get_trans_with_utime( self->bot_frames , "CAMERA",  "local", msg->utime, cam_to_local);
  status_body = self->bot_frames_cpp->get_trans_with_utime(  self->bot_frames , "CAMERA",  "body", msg->utime, cam_to_body);

  if ((0 == status_local) ||(0 == status_body  )) {
    std::cerr << "SensorDataReceiver: cannot get transform from CAMERA to frame" << std::endl;
  }

//  Eigen::Vector4d pos_vec1= Eigen::Vector4d(goal_pos[0], goal_pos[1], goal_pos[2], 1);
  self->local_pointpose_to  =  cam_to_local* Eigen::Vector4d(goal_pos[0], goal_pos[1], goal_pos[2], 1);
  self->local_pointpose_from=  cam_to_local* Eigen::Vector4d(0,0,0, 1);
  
  Eigen::Vector4d pos_vec= Eigen::Vector4d(goal_pos[0], goal_pos[1], goal_pos[2], 1);
  self->body_pointpose_to  =  cam_to_body*pos_vec;
  Eigen::Vector4d null_vec= Eigen::Vector4d(0,0,0, 1);
  self->body_pointpose_from=  cam_to_body*null_vec;

  cout << "bottom\n";
  
  
  /*
  self->robot_utime =msg->utime;
  
  // Remove the pose roll and pitch:
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->origin_position.rotation.w, msg->origin_position.rotation.x, 
                                               msg->origin_position.rotation.y, msg->origin_position.rotation.z);
  double ypr[3];
  quat_to_euler(quat, ypr[0], ypr[1], ypr[2]);
  ypr[1] =0; ypr[2] =0;
  Eigen::Quaterniond quat_yaw_only = euler_to_quat(ypr[0], ypr[1], ypr[2]);
  self->robot_pose.setIdentity();
  self->robot_pose.translation()  << msg->origin_position.translation.x, msg->origin_position.translation.y, msg->origin_position.translation.z;
  self->robot_pose.rotate(quat_yaw_only);    
  self->robot_yaw = ypr[0]; // for convenece keep the yaw

  // Determine the left and right max turning circles:
  Eigen::Isometry3d offset;
  offset.setIdentity();
  offset.translation()  << 0, self->min_turn_radius,0;
  self->center_arc_left = self->robot_pose * offset;
  offset.translation()  << 0,- (self->min_turn_radius) ,0;
  self->center_arc_right = self->robot_pose * offset;
  bot_viewer_request_redraw(self->viewer);*/
}



static void
_free (BotRenderer *renderer)
{
  free (renderer);
}

BotRenderer *renderer_driving_new (BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param, BotFrames * frames)
{
  RendererDriving *self = new RendererDriving();
//  RendererDriving *self = (RendererDriving*) calloc (1, sizeof (RendererDriving));
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
  self->bot_param = param;
  self->bot_frames = frames;  
  self->robot_pose.setIdentity();
  
  self->heightmap_res = HEIGHTMAP_RES_LOW;
  self->visual_goal_type = VISUAL_GOAL_GLOBAL;
  
  self->goal_timeout =5.0;
  self->min_turn_radius =4.0; /// Assumed Example: http://www.atv.com/manufacturers/john-deere/2011-john-deere-gator-xuv-825i-4x4-review-1798.html
  self->wheel_separation = 1.5; // metres
  self->max_velocity = 2.0; // m/s  - about 5m/s
  self->center_arc_left.setIdentity();
  self->center_arc_right.setIdentity();
  
  // TODO: set these to null at init:
  // self->local_pointpose_from | self->local_pointpose_to
  
  drc_robot_state_t_subscribe(self->lc,"EST_ROBOT_STATE",on_est_robot_state,self); 
  perception_pointing_vector_t_subscribe(self->lc,"OBJECT_BEARING",on_pointing_vector,self); 
  
  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  bot_gtk_param_widget_add_double(self->pw, PARAM_GOAL_TIMEOUT, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 30.0, .5, 5.0);  
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_GOAL_SEND, NULL);
  
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_GOAL_SEND_LEFT_HAND, NULL);
  bot_gtk_param_widget_add_enum(self->pw, PARAM_VISUAL_GOAL_TYPE, BOT_GTK_PARAM_WIDGET_MENU, VISUAL_GOAL_GLOBAL, "Global", VISUAL_GOAL_GLOBAL, "Relative", VISUAL_GOAL_RELATIVE, NULL);
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_VISUAL_GOAL, NULL);

  bot_gtk_param_widget_add_buttons(self->pw, PARAM_NEW_MAP, NULL);
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_NEW_OCTOMAP, NULL);
  bot_gtk_param_widget_add_enum(self->pw, PARAM_HEIGHTMAP_RES, BOT_GTK_PARAM_WIDGET_MENU, HEIGHTMAP_RES_LOW, "High", HEIGHTMAP_RES_HIGH, "Low", HEIGHTMAP_RES_LOW, NULL);
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_UPDATE_HEIGHTMAP, NULL);

  bot_gtk_param_widget_add_double(self->pw, PARAM_LIDAR_RATE, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 1.0, 0.025, 0.25);  
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_LIDAR_RATE_SEND, NULL);
  
  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  self->renderer.widget = GTK_WIDGET(self->pw);

  self->active = 0;

  return &self->renderer;
}

void setup_renderer_driving(BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param,
    BotFrames * frames)
{
  bot_viewer_add_renderer_on_side(viewer, renderer_driving_new(viewer, render_priority, lcm, param, frames),
      render_priority , 0);
}
