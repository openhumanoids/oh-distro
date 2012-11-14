/*
 * Renders a set of scrolling plots in the top right corner of the window
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <iostream>

#include <GL/gl.h>
#include <GL/glu.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_core/bot_core.h>
#include <lcmtypes/drc_heightmap_t.h>

using namespace std;

#define PARAM_COLOR_MODE "Color Mode"
#define PARAM_COLOR_MODE_Z_MAX_Z "Red Height"
#define PARAM_COLOR_MODE_Z_MIN_Z "Blue Height"

#define PARAM_HEIGHT_MODE "Height Mode"

#define PARAM_COLOR_ALPHA "Alpha"
#define PARAM_NAME_FREEZE "Freeze"

#define RENDERER_NAME "Heightmap"


typedef enum _color_mode_t {
  COLOR_MODE_RANGE, COLOR_MODE_Z, COLOR_MODE_ORANGE, COLOR_MODE_GRADIENT
} color_mode_t;

typedef enum _height_mode_t {
  HEIGHT_MODE_MESH, HEIGHT_MODE_WIRE, HEIGHT_MODE_NONE,
} height_mode_t;

typedef struct _RendererHeightmap RendererHeightmap;

struct _RendererHeightmap {
    BotRenderer renderer;
    BotViewer *viewer;
    lcm_t *lcm;
    BotGtkParamWidget    *pw;
    drc_heightmap_t *hmap;
    double zMin;
    double zMax;

    // Example Cost Map:
    drc_heightmap_t *cmap;
    
    height_mode_t height_mode;
    color_mode_t color_mode;
    double color_alpha;
};


static void on_costmap(const lcm_recv_buf_t * buf, const char *channel, const drc_heightmap_t *msg, void *user_data){
    RendererHeightmap *self = (RendererHeightmap*) user_data;
  if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;
  self->cmap = drc_heightmap_t_copy(msg);
  bot_viewer_request_redraw(self->viewer);
}

static void on_heightmap(const lcm_recv_buf_t * buf, const char *channel, const drc_heightmap_t *msg, void *user_data){
    RendererHeightmap *self = (RendererHeightmap*) user_data;
  
  if (bot_gtk_param_widget_get_bool (self->pw, PARAM_NAME_FREEZE)) return;
  self->hmap = drc_heightmap_t_copy(msg);
  bot_viewer_request_redraw(self->viewer);
}

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, 
        RendererHeightmap *self)
{

  self->zMax = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z);
  self->zMin = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z);

  self->height_mode =(height_mode_t)  bot_gtk_param_widget_get_enum(self->pw, PARAM_HEIGHT_MODE);
  self->color_mode =(color_mode_t)  bot_gtk_param_widget_get_enum(self->pw, PARAM_COLOR_MODE);
  self->color_alpha =  bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_ALPHA);
  
  bot_viewer_request_redraw(self->viewer);
}


static void draw_tri(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c,
		     RendererHeightmap *self, int index){
  double norm_val;
  float * outC; //rgb colour out
  if (self->color_mode== COLOR_MODE_Z){
    // normalized 0->1 height
    norm_val = ( a(2) - self->zMin) / (self->zMax - self->zMin);
    outC = bot_color_util_jet(norm_val);
  }else if(self->color_mode == COLOR_MODE_RANGE){
    norm_val = sqrt(  pow( a(0),2) + pow( a(1),2) + pow( a(2),2) ) /5.0;
    outC = bot_color_util_jet(norm_val);
  }else if(self->color_mode == COLOR_MODE_ORANGE){
    float orange[] ={1.0, 0.5, 0.0};
    outC = orange;
  }else if(self->color_mode == COLOR_MODE_GRADIENT){
    if (self->cmap == NULL){
      //cout << "no cmap\n";
      float orange[] ={1.0, 0.5, 0.0};
      outC = orange;
    }else{
      //cout << "got cmap\n";
      norm_val =  self->cmap->heights[index] ;
      outC = bot_color_util_jet(norm_val);
    }
  }else {
    norm_val = 0.8;
    outC = bot_color_util_jet(norm_val);
  }
  
  if (self->height_mode== HEIGHT_MODE_MESH){
    glBegin(GL_TRIANGLES);
      glColor4f(outC[0], outC[1], outC[2], self->color_alpha);
      glVertex3f( a(0), a(1), a(2));
      glVertex3f( b(0), b(1), b(2));
      glVertex3f( c(0), c(1), c(2));
    glEnd();
  }else if(self->height_mode== HEIGHT_MODE_WIRE){
    glBegin(GL_LINE_LOOP);
      glColor4f(outC[0], outC[1], outC[2], self->color_alpha);
      glVertex3f( a(0), a(1), a(2));
      glVertex3f( b(0), b(1), b(2));
      glVertex3f( c(0), c(1), c(2));
    glEnd();
  }else if(self->height_mode== HEIGHT_MODE_NONE){
    glBegin(GL_TRIANGLES);
      glColor4f(outC[0], outC[1], outC[2], self->color_alpha);
      glVertex3f( a(0), a(1), self->zMin);
      glVertex3f( b(0), b(1), self->zMin);
      glVertex3f( c(0), c(1), self->zMin);
    glEnd();
  }
  

 
}


static void heightmap_plots_draw (BotViewer *viewer, BotRenderer *renderer)
{
  RendererHeightmap *self = (RendererHeightmap*) renderer->user;
  if (self->hmap == NULL){
    return;
  }
    
  Eigen::Isometry3f mTransformToLocal;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mTransformToLocal(i,j) = ((float)	  self->hmap->transform_to_local[i][j]);
    }
  }  
    
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
//  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glEnable (GL_RESCALE_NORMAL);

  //glTranslatef(0.0f,0.0f,0.0f);	// Move 1.5 Left And 6.0 Into The Screen.
  //glColor4f(0.0,1.0,0.0,0.3);
  //glScalef (1.0, 1.0, 1.0);

    for (int i=0; i < self->hmap->ny -1 ;i++){
    for (int j=0; j < self->hmap->nx -1 ;j++){
          int index = i*self->hmap->nx + j;
          double z00 = self->hmap->heights[index];
          double z10 = self->hmap->heights[index+1];
          double z01 = self->hmap->heights[index+self->hmap->nx];
          double z11 = self->hmap->heights[index+self->hmap->nx+1];
          bool valid00 = z00 > -1e10;
          bool valid10 = z10 > -1e10;
          bool valid01 = z01 > -1e10;
          bool valid11 = z11 > -1e10;
          int validSum = (int)valid00 + (int)valid10 +
            (int)valid01 + (int)valid11;
          if (validSum < 3) {
            continue;
          }

          Eigen::Affine3f xform = mTransformToLocal.cast<float>();
          Eigen::Vector3f p00 = xform*Eigen::Vector3f(j,i,z00);
          Eigen::Vector3f p10 = xform*Eigen::Vector3f(j+1,i,z10);
          Eigen::Vector3f p01 = xform*Eigen::Vector3f(j,i+1,z01);
          Eigen::Vector3f p11 = xform*Eigen::Vector3f(j+1,i+1,z11);
	  
          if (validSum == 4) {
            draw_tri(p00, p10, p01, self, index);
            draw_tri(p11, p10, p01, self, index);
          }	  
          else {
            if (!valid00) {
              draw_tri(p10, p01, p11, self, index);
            }
            else if (!valid10) {
              draw_tri(p00, p01, p11, self, index);
            }
            else if (!valid01) {
              draw_tri(p00, p11, p10, self, index);
            }
            else if (!valid11) {
              draw_tri(p00, p01, p10, self, index);
            }
	  }
      }
    }  
}

static void heightmap_plots_free (BotRenderer *renderer) 
{
    RendererHeightmap *self = (RendererHeightmap*) renderer;
    free (renderer);
}



static void on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererHeightmap *self = (RendererHeightmap*) user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    RendererHeightmap *self = (RendererHeightmap*)  user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, RENDERER_NAME);
}

void heightmap_add_renderer_to_viewer(BotViewer* viewer, int priority, lcm_t* lcm)
{
  RendererHeightmap *self =
      (RendererHeightmap*) calloc (1, sizeof (RendererHeightmap));
  self->viewer = viewer;
  self->renderer.draw = heightmap_plots_draw;
  self->renderer.destroy = heightmap_plots_free;
  self->renderer.name = "Heightmap";
  self->renderer.user = self;
  self->renderer.enabled = 1;

  self->renderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
  //    self->ctrans = globals_get_ctrans();

  //    self->lcm = globals_get_lcm ();
  self->lcm = lcm;

  self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
  gtk_container_add (GTK_CONTAINER (self->renderer.widget),
      GTK_WIDGET(self->pw));
  gtk_widget_show (GTK_WIDGET (self->pw));

  bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, -20, 20.0, .1,  20.0 );
  bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, -20, 20.0, .1,  -20.0 );

  bot_gtk_param_widget_add_enum(self->pw, PARAM_COLOR_MODE, BOT_GTK_PARAM_WIDGET_MENU, COLOR_MODE_Z, 
      "Height", COLOR_MODE_Z, "Range", COLOR_MODE_RANGE, 
      "Gradient", COLOR_MODE_GRADIENT, "Orange", COLOR_MODE_ORANGE, NULL);  

  bot_gtk_param_widget_add_enum(self->pw, PARAM_HEIGHT_MODE, BOT_GTK_PARAM_WIDGET_MENU, HEIGHT_MODE_MESH, "Mesh",
      HEIGHT_MODE_MESH, "Wire", HEIGHT_MODE_WIRE, "None", HEIGHT_MODE_NONE, NULL);  
  
  
  bot_gtk_param_widget_add_double (self->pw, PARAM_COLOR_ALPHA,
      BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, 0.001, 1);
  bot_gtk_param_widget_add_booleans (self->pw,
      BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON, PARAM_NAME_FREEZE, 0, NULL);
  
  g_signal_connect (G_OBJECT (self->pw), "changed",
      G_CALLBACK (on_param_widget_changed), self);


  // save widget modes:
  g_signal_connect (G_OBJECT (viewer), "load-preferences",
      G_CALLBACK (on_load_preferences), self);
  g_signal_connect (G_OBJECT (viewer), "save-preferences",
      G_CALLBACK (on_save_preferences), self);


  drc_heightmap_t_subscribe(self->lcm, "COST_MAP",on_costmap, self);  

  drc_heightmap_t_subscribe(self->lcm, "HEIGHT_MAP",on_heightmap, self);  

  printf("Finished Setting Up Scrolling Plots\n");

  bot_viewer_add_renderer(viewer, &self->renderer, priority);

  return;
}


