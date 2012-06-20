// Renderer to control the display the ground truth map
// mfallon aug2011
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
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

//#include <visualization/renderer_groundtruth.h>

#include <lcmtypes/vs_obj_t.h>
#include <lcmtypes/vs_obj_collection_t.h>
#include <lcmtypes/vs_localize_reinitialize_cmd_t.h>
#include <lcmtypes/bot_core.h>




#define RENDERER_NAME "GroundTruth"
#define PARAM_REMOVE_WALL "Remove Wall"

#define PARAM_START_TRIM "Start Trim"
#define PARAM_A_PLUS "A+"
#define PARAM_A_NEG "A-"
#define PARAM_B_PLUS "B+"
#define PARAM_B_NEG "B-"
#define PARAM_END_TRIM "End Trim"
#define PARAM_SCALE "Scaling"

#define PARAM_SAVE_DB "Save DB"

#define PARAM_YAW "Yaw"
#define PARAM_X "X"
#define PARAM_Y "Y"

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


int geom_ray_plane_intersect_3d_gt (const point3d_t *ray_point, const vec3d_t *ray_dir,
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

int geom_ray_z_plane_intersect_3d_gt(const point3d_t *ray_point, 
        const point3d_t *ray_dir, double plane_z, point2d_t *result_xy)
{
    point3d_t plane_pt = { 0, 0, plane_z};
    point3d_t plane_normal = { 0, 0, 1};
    point3d_t plane_isect_point;
    double plane_point_dist;
    if (!geom_ray_plane_intersect_3d_gt (ray_point, ray_dir, &plane_pt, 
                &plane_normal, &plane_isect_point, &plane_point_dist) ||
        plane_point_dist <= 0) {
        return -1;
    }
    result_xy->x = plane_isect_point.x;
    result_xy->y = plane_isect_point.y;
    return 0;
}


////////////////////////////// END OF CODE COPIED IN FROM COMMON_UTILS

typedef struct _RendererLocalize {
    BotRenderer renderer;
    BotEventHandler ehandler;
    BotViewer *viewer;
    lcm_t *lc;

    BotGtkParamWidget *pw;

    int dragging;
    int active; //1 = relocalize, 2 = set person location 
    point2d_t drag_start_local;
    point2d_t drag_finish_local;

    point2d_t particle_mean;
    double theta;
    double particle_std;

    int64_t max_draw_utime;
}RendererLocalize;

static void
_draw (BotViewer *viewer, BotRenderer *renderer)
{
    RendererLocalize *self = (RendererLocalize*) renderer;
    int64_t now = bot_timestamp_now();
    if(!self->dragging && now > self->max_draw_utime)
        return;

    glColor3f(0, 1, 0);
    glPushMatrix();
    glTranslatef(self->particle_mean.x, self->particle_mean.y, 0);
    bot_gl_draw_circle(self->particle_std);
    //bot_gl_draw_arrow_2d();

    glBegin(GL_LINE_STRIP);  
    glVertex2f(0.0,0.0);

    glVertex2f(self->particle_std*cos(self->theta),self->particle_std*sin(self->theta));
    glEnd();

    glPopMatrix();
}

static void
recompute_particle_distribution(RendererLocalize *self)
{
  
    self->particle_mean = self->drag_start_local;
    double dx = self->drag_finish_local.x - self->drag_start_local.x;
    double dy = self->drag_finish_local.y - self->drag_start_local.y;

    double theta = atan2(dy,dx);
    self->theta = theta;

    self->particle_std = sqrt(dx*dx + dy*dy);
    if(self->particle_std < MIN_STD)
        self->particle_std = MIN_STD;
    if(self->particle_std > MAX_STD)
        self->particle_std = MAX_STD;
    self->max_draw_utime = bot_timestamp_now() + DRAW_PERSIST_SEC * 1000000;
    
}

static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
             const double ray_dir[3], const GdkEventButton *event)
{
    RendererLocalize *self = (RendererLocalize*) ehandler->user;

    //fprintf(stderr, "Mouse Press : %f,%f\n", ray_start[0], ray_start[1]);

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
  
    if (0 != geom_ray_z_plane_intersect_3d_gt(POINT3D(ray_start), 
                                           POINT3D(ray_dir), 0, &click_pt_local)) {
        bot_viewer_request_redraw(self->viewer);
        self->active = 0;
        return 0;
    }

    self->dragging = 1;

    self->drag_start_local = click_pt_local;
    self->drag_finish_local = click_pt_local;

    recompute_particle_distribution(self);

    bot_viewer_request_redraw(self->viewer);
    return 1;
}



static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3], 
                         const GdkEventButton *event)
{
    RendererLocalize *self = (RendererLocalize*) ehandler->user;

    if (self->dragging) {
        self->dragging = 0;
    }
    if (self->active != 0) {
        // check drag points and publish
        
        printf("x,y,t: %f %f %f.    std: %f\n",self->particle_mean.x
	    ,self->particle_mean.y,self->theta,self->particle_std); 
        
  	vs_localize_reinitialize_cmd_t msg;
        msg.utime = bot_timestamp_now();
        msg.mean[0] = self->particle_mean.x;
        msg.mean[1] = self->particle_mean.y;
        msg.mean[2] = self->theta;
      
        double v = self->particle_std * self->particle_std;
        msg.variance[0] = v;
        msg.variance[1] = v;
        msg.variance[2] = VARIANCE_THETA;
      
        fprintf(stderr,"Button Released => Activate Value : %d\n", self->active);
        if(self->active == 1){
            fprintf(stderr, "GT_REMOVE_WALL sent\n");
            vs_localize_reinitialize_cmd_t_publish(self->lc, "GT_REMOVE_WALL", &msg);
        }else if (self->active ==2){
            fprintf(stderr, "GT_START_TRIM sent\n");
            vs_localize_reinitialize_cmd_t_publish(self->lc, "GT_START_TRIM", &msg);
	}
        bot_viewer_set_status_bar_message(self->viewer, "sent msg");
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
    RendererLocalize *self = (RendererLocalize*) ehandler->user;

    if(!self->dragging || self->active==0)
        return 0;

    point2d_t drag_pt_local;
    if (0 != geom_ray_z_plane_intersect_3d_gt(POINT3D(ray_start), 
                                           POINT3D(ray_dir), 0, &drag_pt_local)) {
        return 0;
    }

    self->drag_finish_local = drag_pt_local;
    recompute_particle_distribution(self);

    bot_viewer_request_redraw(self->viewer);
    return 1;
}

void activate_gt(RendererLocalize *self, int type)
{
    self->active = type;
//     if(type==1){
//         bot_viewer_set_status_bar_message(self->viewer, 
//                                           "Click and drag to initialize new particles");
//     }
//     if(type==2){
//         bot_viewer_set_status_bar_message(self->viewer, 
//                                           "Click and drag to initialize new person");
//     }
}

static int key_press (BotViewer *viewer, BotEventHandler *ehandler, 
                      const GdkEventKey *event)
{
    RendererLocalize *self = (RendererLocalize*) ehandler->user;

    if ((event->keyval == 'l' || event->keyval == 'L') && self->active==0) {
        activate_gt(self,1);
        bot_viewer_request_pick (viewer, ehandler);
    } else if(event->keyval == GDK_Escape) {
        //self->active = 0;
        //ehandler->picking = 0;
        //bot_viewer_set_status_bar_message(self->viewer, "");
    }

    return 0;
}


// Send a message to indicate which end of the line to trim
// change of A and B packed into a single message
// this is really lasy as it reused the reinitialse message
// i did this to avoid inventing another
static void send_change (RendererLocalize *self, int which, int direction){
  BotViewHandler *vhandler = self->viewer->view_handler;  
  
  vs_localize_reinitialize_cmd_t msg;
  msg.utime = bot_timestamp_now();
  msg.mean[0] = -99999;
  msg.mean[1] = -99999;
  double scale_amount = direction*bot_gtk_param_widget_get_double (self->pw, PARAM_SCALE);
  if (which ==1){ // change a
    fprintf(stderr,"Sent GT_CHANGE LCM message. A %d\n", scale_amount);
    msg.mean[0] = scale_amount;
  }else if (which ==2){ // change b
    fprintf(stderr,"Sent GT_CHANGE LCM message. B %d\n", scale_amount);
    msg.mean[1] = scale_amount;
  }
  msg.mean[2] = -99999;
  msg.variance[0] = -99999;
  msg.variance[1] = -99999;
  msg.variance[2] = -99999;
  vs_localize_reinitialize_cmd_t_publish(self->lc, "GT_CHANGE", &msg);  
}



// Send a message to save the relocalize BOW DB
// this is really lasy as it reused the reinitialse message
// i did this to avoid inventing another
static void send_translation (RendererLocalize *self){
  BotViewHandler *vhandler = self->viewer->view_handler;  
  vs_obj_collection_t objs;
  objs.id = 0; 
  objs.name =(char*) "Map Translation"; // "Trajectory";
  objs.type = 1; // a pose
  objs.reset = false; // true will delete them from the viewer
  objs.nobjs = 1;
  vs_obj_t poses_x[objs.nobjs];
  poses_x[0].id = bot_timestamp_now();
  poses_x[0].x = bot_gtk_param_widget_get_double (self->pw, PARAM_X);
  poses_x[0].y = bot_gtk_param_widget_get_double (self->pw, PARAM_Y);
  poses_x[0].z = 0;
  poses_x[0].yaw = M_PI*bot_gtk_param_widget_get_double (self->pw, PARAM_YAW)/180;
  poses_x[0].pitch = 0;
  poses_x[0].roll = 0;
  objs.objs = poses_x;
  
  
  vs_obj_collection_t_publish(self->lc, "GT_OBJ_COLLECTION", &objs); 
}



// Send a message to save the relocalize BOW DB
// this is really lasy as it reused the reinitialse message
// i did this to avoid inventing another
static void end_trim (RendererLocalize *self){
  BotViewHandler *vhandler = self->viewer->view_handler;  

  vs_localize_reinitialize_cmd_t msg;
  msg.utime = bot_timestamp_now();
  msg.mean[0] = -99999;
  msg.mean[1] = -99999;
  msg.mean[2] = -99999;
  msg.variance[0] = -99999;
  msg.variance[1] = -99999;
  msg.variance[2] = -99999;
  //if(self->active == 1){
      fprintf(stderr,"Sent GT_END_TRIM LCM message %d\n", self->active);
      vs_localize_reinitialize_cmd_t_publish(self->lc, "GT_END_TRIM", &msg);
  //}      
//        self->active = 0;
}




// Send a message to save the relocalize BOW DB
// this is really lasy as it reused the reinitialse message
// i did this to avoid inventing another
static void save_db (RendererLocalize *self){
  BotViewHandler *vhandler = self->viewer->view_handler;  

  vs_localize_reinitialize_cmd_t msg;
  msg.utime = bot_timestamp_now();
  msg.mean[0] = -99999;
  msg.mean[1] = -99999;
  msg.mean[2] = -99999;
  msg.variance[0] = -99999;
  msg.variance[1] = -99999;
  msg.variance[2] = -99999;
  //if(self->active == 1){
      fprintf(stderr,"Sent GT_SAVE LCM message %d\n", self->active);
      vs_localize_reinitialize_cmd_t_publish(self->lc, "GT_SAVE", &msg);
  //}      
//        self->active = 0;
}


static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
    RendererLocalize *self = (RendererLocalize*) user;
    if(!strcmp(name, PARAM_REMOVE_WALL)) {
        fprintf(stderr,"Clicked reinit, activate\n");
        bot_viewer_request_pick (self->viewer, &(self->ehandler));
        activate_gt(self, 1);
    }else if(!strcmp(name, PARAM_START_TRIM)) {
        fprintf(stderr,"Clicked goal, activate\n");
        bot_viewer_request_pick (self->viewer, &(self->ehandler));
        activate_gt(self, 2);
    }else if (! strcmp (name, PARAM_END_TRIM)) {
	end_trim(self);
    }else if (! strcmp (name, PARAM_A_NEG)) {
        fprintf(stderr,"Clicked a neg, activate\n");
        send_change(self,1,-1);
    }else if (! strcmp (name, PARAM_A_PLUS)) {
        fprintf(stderr,"Clicked a plus, activate\n");
        send_change(self,1,1);
    }else if (! strcmp (name, PARAM_B_NEG)) {
        fprintf(stderr,"Clicked b neg, activate\n");
        send_change(self,2,-1);
    }else if (! strcmp (name, PARAM_B_PLUS)) {
        fprintf(stderr,"Clicked b plus, activate\n");
        send_change(self,2,1);
    }else if (! strcmp (name, PARAM_SAVE_DB)) {
        save_db(self);
    }else if (! strcmp (name, PARAM_X)) {
        fprintf(stderr,"Clicked x, activate\n");
        send_translation(self);
    }else if (! strcmp (name, PARAM_Y)) {
        fprintf(stderr,"Clicked y, activate\n");
        send_translation(self);
    }else if (! strcmp (name, PARAM_YAW)) {
        fprintf(stderr,"Clicked yaw, activate\n");
        send_translation(self);
    }
}

static void
_free (BotRenderer *renderer)
{
    free (renderer);
}

BotRenderer *renderer_groundtruth_new (BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    RendererLocalize *self = (RendererLocalize*) calloc (1, sizeof (RendererLocalize));
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
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_REMOVE_WALL, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_START_TRIM, NULL);
    
    bot_gtk_param_widget_add_double (self->pw, PARAM_SCALE, 
            BOT_GTK_PARAM_WIDGET_SLIDER, 0, 2, 0.01, 0);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_A_NEG, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_A_PLUS, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_B_NEG, NULL);
    bot_gtk_param_widget_add_buttons(self->pw, PARAM_B_PLUS, NULL);
    
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON,
            PARAM_END_TRIM, 0, NULL);
    
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON,
            PARAM_SAVE_DB, 0, NULL);
    
    bot_gtk_param_widget_add_double (self->pw, PARAM_YAW, 
            BOT_GTK_PARAM_WIDGET_SLIDER, -180, 180, 0.1, 0);
    bot_gtk_param_widget_add_double (self->pw, PARAM_X, 
            BOT_GTK_PARAM_WIDGET_SLIDER, -200, 200, 0.1, 0);
    bot_gtk_param_widget_add_double (self->pw, PARAM_Y, 
            BOT_GTK_PARAM_WIDGET_SLIDER, -200, 200, 0.1, 0);
    
    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    self->renderer.widget = GTK_WIDGET(self->pw);

    self->active = 0;

    return &self->renderer;
}

void setup_renderer_groundtruth(BotViewer *viewer, int render_priority, lcm_t *lcm)
{
    bot_viewer_add_renderer(viewer, renderer_groundtruth_new(viewer, render_priority, lcm), 
                            render_priority);
}
