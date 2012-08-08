/*
 * Orders a flythrough of the viewer using the obj_collection type
 * Note: required to publish pitchroll bullshit at present as
 * it doesnt seem to renderer properly if it doesnt publish at least
 * something graphical... i think
 * mfallon mar 2011
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <iostream>
#include <GL/gl.h>
#include <GL/glu.h>

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_core/bot_core.h>

#include "visualization/collections.hpp"

#include "sort.h"

#define PARAM_NAME_DURATION "Duration"
#define PARAM_NAME_SHOW_PANORAMA "Show Panorama"
#define PARAM_NAME_SHOW_FLYTHROUGH "Show Flythrough"
#define PARAM_NAME_RENDER_PITCHROLL ""
#define PARAM_WHICH_COLL "Coll No"

// max number of points on the scrolling bar:
#define MAX_POINTS 1000

#define RENDERER_NAME "Scrolling_Plots"
#define PARAM_MODE "Mode"

typedef struct _RendererPanorama RendererPanorama;

using namespace std;

struct _RendererPanorama {
  BotRenderer renderer;
  BotEventHandler ehandler;
  BotViewer *viewer;
  lcm_t *lcm;

  BotGtkParamWidget    *pw;
  BotGlScrollPlot2d *pitchroll_plot;

  vs_obj_collection_t* obj_coll;
  int do_flythrough;
  int flythrough_index;
  int64_t flythrough_start_utime;
  std::vector<size_t> reordering;

  // for mouse clicks:
  double last_xy[2];   // Used moving and resizing operating, goal, and obstacle regions



  uint64_t      max_utime;
};

enum {
  MODE_EYE,
  MODE_BEHIND,
  MODE_MOREBEHIND,
  MODE_ABOVE,
};

static void save_panorama (RendererPanorama *self);

static void on_obj_coll(const lcm_recv_buf_t * buf, const char *channel, const vs_obj_collection_t *msg, 
    void *user_data);

static void update_xaxis (RendererPanorama *self, uint64_t utime);

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, 
    RendererPanorama *self);

static void scrolling_plots_draw (BotViewer *viewer, BotRenderer *renderer)
{
  RendererPanorama *self = (RendererPanorama*) renderer->user;
  if (!self->max_utime) return;

  if (self->do_flythrough){
    BotViewHandler *vhandler = self->viewer->view_handler;  

    int fly_method = MODE_ABOVE; // default..
    int flythrough_time_total = 10*1000000; // 10 seconds
    if (self->viewer) {
      fly_method = bot_gtk_param_widget_get_enum (self->pw, PARAM_MODE);
      flythrough_time_total = bot_gtk_param_widget_get_int (self->pw, PARAM_NAME_DURATION) *1000000;
    }

    double eye[3],lookat[3],up[3];
    vs_obj_collection_t* objc = self->obj_coll;

    double dist_behind = 0.01; // number of meters to put the eye behind
    double dist_behind_above =0; //and an additional bit above	  
    if (fly_method==MODE_EYE){ // at the obj
      dist_behind=0.01;
      dist_behind_above = 0;
    }else if (fly_method==MODE_BEHIND){ //  a little back (good for tight motion
      dist_behind=1.5;
      dist_behind_above = 0.5;
    }else if (fly_method==MODE_MOREBEHIND){ // a lot back (good for perspective)
      dist_behind=3.5;
      dist_behind_above = 1;
    }else if (fly_method==MODE_ABOVE){ // above with fixed rotation
      dist_behind=0;
      dist_behind_above = 0;
    }

    int64_t current_utime = bot_timestamp_now();
    double fly_fraction = ((double) (current_utime - self->flythrough_start_utime)) /((double)flythrough_time_total);

    if (fly_fraction > 1){
      printf("Finished flythrough\n");
      self->do_flythrough=0; 
    }else{
      int64_t coll_start_utime = objc->objs[self->reordering[0]].id;
      int64_t coll_end_utime = objc->objs[self->reordering[self->obj_coll->nobjs-1]].id;

      int64_t current_fly_utime =  (1-fly_fraction)*coll_start_utime   + fly_fraction* coll_end_utime;

      int found_id=0;
      int w_id =0;
      while(found_id==0){
        int w_id_ordered = self->reordering[w_id];
        if (current_fly_utime < objc->objs[w_id_ordered].id ){
          found_id =1;
          break;
        }else if (w_id > objc->nobjs){
          break;
        }
        w_id++;
      }

      if (found_id){
        // 1. Interpolate the transform between to successive objs and use that as the
        // basis of the eye and lookat positions
        int j1 = self->reordering[w_id-1];
        int j2 = self->reordering[w_id];

        // 1.2 fraction of time between successive objs
        double frac = ((double)current_fly_utime - objc->objs[j1].id) / ((double) objc->objs[j2].id  - objc->objs[j1].id);
        double rpy1[3],rpy2[3],rpy_interp[3];
        rpy1[0] = objc->objs[j1].roll;
        rpy1[1] = objc->objs[j1].pitch;
        rpy1[2] = objc->objs[j1].yaw;
        rpy2[0] = objc->objs[j2].roll;
        rpy2[1] = objc->objs[j2].pitch;
        rpy2[2] = objc->objs[j2].yaw;
        double q1[4],q2[4],q_interp[4];
        bot_roll_pitch_yaw_to_quat(rpy1,  q1  );
        bot_roll_pitch_yaw_to_quat(rpy2,  q2 );
        bot_quat_interpolate( q1, q2, frac,q_interp);
        bot_quat_to_roll_pitch_yaw(q_interp,rpy_interp  );

        // 1.3 Given the Interpolation, place the eye behind the lookat, as requested:
        double pitch = rpy_interp[1];
        double yaw = rpy_interp[2];
        double behind[3]={0};
        behind[0] = dist_behind *cos(pitch) * cos(yaw);
        behind[1] = dist_behind* cos(pitch) * sin(yaw);
        behind[2] = dist_behind*sin(pitch);
        //cout << "interp ypr " << rpy_interp[2] << ", " << rpy_interp[1] << ", " << rpy_interp[0]  << "\n";
        //cout << behind[0] << ", " << behind[1] << ", " << behind[2] << "\n";
        // add interpolation
        lookat[0] = (1-frac)*objc->objs[j1].x + frac*objc->objs[j2].x;
        lookat[1] = (1-frac)*objc->objs[j1].y + frac*objc->objs[j2].y;
        //lookat[2] = (1-frac)*(objc->objs[j1].z + 1.0) + frac*(objc->objs[j2].z + 1.0);
        lookat[2] = (1-frac)*(objc->objs[j1].z + 0.0) + frac*(objc->objs[j2].z + 0.0);
        eye[0] = lookat[0] - behind[0];
        eye[1] = lookat[1] - behind[1];
        eye[2] = lookat[2] - behind[2];
        up[0] = 0;
        up[1] = 0;
        up[2] = 2*M_PI;
        // add a nudge above - if required to
        eye[2] = eye[2] +dist_behind_above;

        // if 'bombing run': follow from above:
        if (fly_method==MODE_ABOVE){
          lookat[0] = eye[0];
          lookat[1] = eye[1];
          lookat[2] = eye[2];
          eye[2] = eye[2] +10;
          up[0] = 0;
          up[1] = 1;
          up[2] = 0;
        }

        vhandler->set_look_at(vhandler, eye, lookat, up);
        //	  gtk_widget_queue_draw (GTK_WIDGET(self->viewer->gl_area));
        //bot_viewer_request_redraw(self->viewer);
        gtk_widget_draw(GTK_WIDGET(self->viewer->gl_area),NULL);

      }
    }

  }
}



#define GEOM_EPSILON 1e-9

#ifndef _point2d_t_h // XXX hack... _point2d_t_h is defined in
// lcmtypes/point2d_t.h
// double
typedef struct _point2d {
  double x;
  double y;
} point2d_t;
#endif

#define point2d_as_array(p) ((double*)p)

/* The magic below allows you to use the POINT2D() macro to convert a
 * double[2] to a point2d_t.  gcc is smart -- if you try to cast anything
 * other than a point2d_t or a double[] with this macro, gcc will emit
 * a warning. */
union _point2d_any_t {
  point2d_t point;
  double array[2];
};
#define POINT2D(p) (&(((union _point2d_any_t *)(p))->point))

// double 
typedef struct _point3d {
  double x;
  double y;
  double z;
} point3d_t;

typedef point3d_t vec3d_t;

union _point3d_any_t {
  point3d_t point;
  double array[3];
};
#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))

typedef struct _plane {
  double a;
  double b;
  double c;
  double d;
} plane4d_t;
typedef plane4d_t vec4d_t;


static double
geom_vec_vec_dot_3d (const vec3d_t *a, const vec3d_t *b)
{
  return a->x * b->x + a->y * b->y + a->z * b->z;
}

#define plane4d_as_array(p) ((double*)p)
#define plane4d_get_normal(p) (POINT3D(plane4d_as_array(p)))

static int
geom_ray_plane_intersect (const point3d_t *ray_point, const vec3d_t *ray_vec,
    const plane4d_t *plane, point3d_t *result, double *u)
{
  double lambda1 = geom_vec_vec_dot_3d (plane4d_get_normal(plane), ray_vec);

  // check for degenerate case where ray is (more or less) parallel to plane
  if (fabs (lambda1) < GEOM_EPSILON) return 0;

  double lambda2 = geom_vec_vec_dot_3d (plane4d_get_normal(plane),
      ray_point) + plane->d;
  double v = lambda2 / -lambda1;
  result->x = ray_point->x + v * ray_vec->x;
  result->y = ray_point->y + v * ray_vec->y;
  result->z = ray_point->z + v * ray_vec->z;
  if (u) *u = v;
  return 1;
}



static  int
geom_ray_z_plane_intersect_3d (const point3d_t *ray_point, 
    const point3d_t *ray_vec, double plane_z, point2d_t *result_xy)
{
  plane4d_t plane = {0, 0, 1, -plane_z};
  point3d_t plane_isect_point;
  double plane_point_dist;
  if (!geom_ray_plane_intersect (ray_point, ray_vec, &plane,
      &plane_isect_point, &plane_point_dist) ||
      plane_point_dist <= 0) {
    return -1;
  }
  result_xy->x = plane_isect_point.x;
  result_xy->y = plane_isect_point.y;
  return 0;
}

static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler,
    const double ray_start[3], const double ray_dir[3],
    const GdkEventButton *event)
{

  RendererPanorama *self = (RendererPanorama *)ehandler->user;

  double xy[2];
  /*   printf("st: %f %f %f dir: %f %f %f\n",
      ray_start[0],ray_start[1],ray_start[2],
      ray_dir[0],ray_dir[1],ray_dir[2]
    );*/
  geom_ray_z_plane_intersect_3d(POINT3D(ray_start), POINT3D(ray_dir),
      0, POINT2D(xy));

  int control = event->state & GDK_CONTROL_MASK;
  //     int shift = event->state & GDK_SHIFT_MASK;

  if ((fabs(xy[0]) <0.1) && (fabs(xy[1]) <0.1) ){
    return 0;
    //printf("moouse xy: %f %f\n",xy[0],xy[1]);
  }

  // only handle control key pressed.
  if (!control)
    return 0;

  self->last_xy[0] = xy[0];
  self->last_xy[1] = xy[1];

  // self->mouseNodeXY[0] = xy[0];
  // self->mouseNodeXY[1] = xy[1];
  // printf("mouse click: xy: %f %f registered for panorama\n",xy[0],xy[1]);

  bot_viewer_request_redraw(self->viewer);
  return 1;
}




static void
scrolling_plots_free (BotRenderer *renderer) 
{
  RendererPanorama *self = (RendererPanorama*) renderer;
  //    globals_release_ctrans (self->ctrans);
  //    globals_release_lcm (self->lcm);
  free (renderer);
}


static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererPanorama *self = (RendererPanorama*) user_data;
  bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererPanorama *self = (RendererPanorama*) user_data;
  bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}


//BotRenderer *renderer_scrolling_plots_new (BotViewer *viewer)
//, BotFrames * frames, const char * kinect_frame)
void panorama_add_renderer_to_viewer(BotViewer* viewer, int priority, lcm_t* lcm)
{
  RendererPanorama *self =
      (RendererPanorama*) calloc (1, sizeof (RendererPanorama));
  self->viewer = viewer;
  self->renderer.draw = scrolling_plots_draw;
  self->renderer.destroy = scrolling_plots_free;
  self->renderer.name = "Panorama";
  self->renderer.user = self;
  self->renderer.enabled = 1;
  self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);


  BotEventHandler *ehandler = &(self->ehandler);
  memset(ehandler, 0x00, sizeof(BotEventHandler));
  ehandler->name = RENDERER_NAME;
  ehandler->enabled = 1;
  ehandler->mouse_press = mouse_press;
  //    ehandler->mouse_motion = mouse_motion;
  ehandler->mouse_release = NULL;
  ehandler->mouse_scroll = NULL;
  ehandler->pick_query = NULL;
  ehandler->hover_query = NULL;
  //    ehandler->key_press = key_press;
  ehandler->user = self;

  self->last_xy[0] =0;
  self->last_xy[1] = 1;

  self->do_flythrough =0;
  self->flythrough_index = 0;

  self->lcm = lcm;

  self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
  gtk_container_add (GTK_CONTAINER (self->renderer.widget),
      GTK_WIDGET(self->pw));
  gtk_widget_show (GTK_WIDGET (self->pw));


  bot_gtk_param_widget_add_int(self->pw,
      PARAM_WHICH_COLL, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 10000, 1, 1);


  bot_gtk_param_widget_add_int (self->pw, PARAM_NAME_DURATION,
      BOT_GTK_PARAM_WIDGET_SLIDER, 2, 120, 2, 20);

  bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON,
      PARAM_NAME_SHOW_FLYTHROUGH, 0, NULL);

  bot_gtk_param_widget_add_enum (self->pw, PARAM_MODE, (BotGtkParamWidgetUIHint)  0,  MODE_EYE,
      "At Object",  MODE_EYE,  "Just Behind", MODE_BEHIND,  "Far Behind",
      MODE_MOREBEHIND,"Above",  MODE_ABOVE, NULL);

  bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON,
      PARAM_NAME_SHOW_PANORAMA, 0, NULL);


  g_signal_connect (G_OBJECT (self->pw), "changed",
      G_CALLBACK (on_param_widget_changed), self);


  // mfallon, save widget modes:
  g_signal_connect (G_OBJECT (viewer), "load-preferences",
      G_CALLBACK (on_load_preferences), self);
  g_signal_connect (G_OBJECT (viewer), "save-preferences",
      G_CALLBACK (on_save_preferences), self);

  // pitchroll plot
  self->pitchroll_plot = bot_gl_scrollplot2d_new ();
  bot_gl_scrollplot2d_set_title        (self->pitchroll_plot, "Pitch [b] & Roll [m]");
  bot_gl_scrollplot2d_set_text_color   (self->pitchroll_plot, 0.7, 0.7, 0.7, 1);
  bot_gl_scrollplot2d_set_bgcolor      (self->pitchroll_plot, 0.1, 0.1, 0.1, 0.7);
  bot_gl_scrollplot2d_set_border_color (self->pitchroll_plot, 1, 1, 1, 0.7);
  bot_gl_scrollplot2d_set_ylim    (self->pitchroll_plot, -10, 10);

  // legends?
  BotGlScrollPlot2dLegendLocation legloc = BOT_GL_SCROLLPLOT2D_HIDDEN;
  bot_gl_scrollplot2d_set_show_legend (self->pitchroll_plot, legloc);

  vs_obj_collection_t_subscribe(self->lcm,"OBJ_COLLECTION",on_obj_coll,self);

  printf("Finished Setting Up Scrolling Plots\n");

  bot_viewer_add_renderer(viewer, &self->renderer, priority);
  bot_viewer_add_event_handler(viewer, ehandler, priority);

}


static void
on_obj_coll(const lcm_recv_buf_t * buf, const char *channel, const vs_obj_collection_t *msg, 
    void *user_data){
  RendererPanorama *self = (RendererPanorama*) user_data;
  if (!self->do_flythrough){ // dont take the new objcollection while flying
    int which_coll = bot_gtk_param_widget_get_int(self->pw, PARAM_WHICH_COLL);
    if (msg->id == which_coll){
      self->obj_coll = vs_obj_collection_t_copy(msg);
    }
  }
  update_xaxis(self,1);
}


static int 
_pixel_convert_8u_bgra_to_8u_rgb(uint8_t *dest, int dstride, int dwidth,
    int dheight, const uint8_t *src, int sstride)
{
  int i, j;
  for (i = 0; i < dheight; i++) {
    uint8_t * drow = dest + i * dstride;
    const uint8_t * srow = src + i * sstride;
    for (j = 0; j < dwidth; j++) {
      drow[j*3 + 0] = srow[j*4 + 2];
      drow[j*3 + 1] = srow[j*4 + 1];
      drow[j*3 + 2] = srow[j*4 + 0];
    }
  }
  return 0;
}

static void 
sort_obj_coll (RendererPanorama *self) {
  BotViewHandler *vhandler = self->viewer->view_handler;

  vs_obj_collection_t* objc = self->obj_coll;
  printf("Starting a flythrough of %d objs... \n",objc->nobjs);

  // sort obj_collection by time: utime_in into utime_out  
  /*// ordering in:
  for (int i=0; i<objc->nobjs;i++){
    printf("changed: %d: %lld  |  %f %f %f  | %f %f %f\n",i,objc->objs[i].id,
	  objc->objs[i].x,objc->objs[i].y,objc->objs[i].z,
	  objc->objs[i].yaw,objc->objs[i].pitch,objc->objs[i].roll);
  }  
  printf("\n");   */

  std::vector<int64_t> utime_in;
  utime_in.resize(objc->nobjs);
  for (int i=0; i<objc->nobjs;i++){
    utime_in[i] = objc->objs[i].id;
  }
  std::vector<size_t> reordering;
  std::vector<int64_t> utime_out;
  sort(utime_in,utime_out,reordering);

  /*//Sorting and ordering out:
  for(int j = 0;j<utime_in.size();j++)
  {
    printf("out[%d] = %lld = in[i[%d]] = in[%d] = %lld\n",j,utime_out[j],j,reordering[j],utime_in[reordering[j]]);
  }
  printf("\n");   
  for (int i=0; i<objc->nobjs;i++){
    int j = reordering[i];
    printf("%d: %lld  |  %f %f %f  | %f %f %f\n",i,objc->objs[j].id,
	  objc->objs[j].x,objc->objs[j].y,objc->objs[j].z,
	  objc->objs[j].yaw,objc->objs[j].pitch,objc->objs[j].roll);
  }  
  printf("\n\n\n");*/

  // Start flying through the points in this order:
  self->reordering = reordering;
  self->do_flythrough =1;
  self->flythrough_index = 0;  
  self->flythrough_start_utime = bot_timestamp_now();
}


// carry out the panaorama at 0,0: save 6 panaoramas files
// to do: use the mouse to get the center point of the panorama
static void save_panorama (RendererPanorama *self){
  BotViewHandler *vhandler = self->viewer->view_handler;

  double eye[3];
  double lookat[3];
  double up[3];
  vhandler->get_eye_look(vhandler, eye, lookat, up);
  double diff[3];
  /*
	printf("original look at: %f %f %f eye: %f %f %f up: %f %f %f\n"
	     ,lookat[0],lookat[1],lookat[2]
	     ,eye[0],eye[1],eye[2]
	     ,up[0],up[1],up[2]);
   */

  double xeye[3],xlookat[3],xup[3];

  int w = GTK_WIDGET (self->viewer->gl_area)->allocation.width;
  int h = GTK_WIDGET (self->viewer->gl_area)->allocation.height;
  printf("save panorama centered on %f %f\n",self->last_xy[0],self->last_xy[1]);
  for (int i=0;i<6;i++){
    uint8_t *bgra = (uint8_t*)malloc (w*h*4);
    uint8_t *rgb = (uint8_t*)malloc (w*h*3);
    xeye[0] = self->last_xy[0];
    xeye[1] = self->last_xy[1];
    xeye[2] = 0;

    xlookat[0] = self->last_xy[0] + cos(i*M_PI/3);
    xlookat[1] = self->last_xy[1] + sin(i*M_PI/3);
    xlookat[2] = 0;
    xup[0] = 0;
    xup[1] = 0;
    xup[2] = 2*M_PI;

    vhandler->set_look_at(vhandler, xeye, xlookat, xup);
    char * fname = bot_fileutils_get_unique_filename (NULL, "viewer", 1, "ppm");
    //	  gtk_widget_queue_draw (GTK_WIDGET(self->viewer->gl_area));
    //bot_viewer_request_redraw(self->viewer);
    gtk_widget_draw(GTK_WIDGET(self->viewer->gl_area),NULL);

    /*
	printf("flook at: %f %f %f eye: %f %f %f up: %f %f %f\n"
	      ,xlookat[0],xlookat[1],xlookat[2]
	      ,xeye[0],xeye[1],xeye[2]
	      ,xup[0],xup[1],xup[2]);
	      printf("w %d h %d\n",w,h);
     */
    glReadPixels (0, 0, w, h, GL_BGRA, GL_UNSIGNED_BYTE, bgra);
    FILE *fp = fopen (fname, "wb");
    if (! fp) {
      perror ("fopen");
      printf ("couldn't take screenshot\n");
      return;
    } else {
      _pixel_convert_8u_bgra_to_8u_rgb (rgb, w*3, w, h, bgra, w*4);
      bot_ppm_write_bottom_up (fp, rgb, w, h, w*3);
      fclose (fp);
      printf ("screenshot saved to %s\n", fname); //was dbg
      //   bot_viewer_set_status_bar_message (self, "screenshot saved to %s", fname);
    }
    free(bgra);
    free(rgb);
    sleep(1);
  }
  // Return to the original view:
  /*
      printf("[end] look at: %f %f %f eye: %f %f %f up: %f %f %f\n"
	,lookat[0],lookat[1],lookat[2]
	,eye[0],eye[1],eye[2]
	,up[0],up[1],up[2]);
   */
  vhandler->set_look_at(vhandler, eye, lookat, up);
  bot_viewer_request_redraw(self->viewer);
  gtk_widget_queue_draw (GTK_WIDGET(self->viewer->gl_area));

}

static void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *name, 
    RendererPanorama *self)
{
  if (! strcmp (name, PARAM_NAME_SHOW_FLYTHROUGH)) {
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_SHOW_FLYTHROUGH)){
      sort_obj_coll(self);
    }  
  }else if (! strcmp (name, PARAM_NAME_SHOW_PANORAMA)) {
    if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_SHOW_PANORAMA)){
      save_panorama(self);
    }
  }
  bot_viewer_request_redraw(self->viewer);
}

static void 
update_xaxis (RendererPanorama *self, uint64_t utime)
{
  self->max_utime = utime;
  double timestamp = self->max_utime * 1e-6;
  bot_gl_scrollplot2d_add_point (self->pitchroll_plot, "25000", timestamp, 2500.0);
}
