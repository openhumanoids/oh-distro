#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include "mav_mapping_renderers.h"

#define RENDERER_NAME "PointCloudRegistration"
#define PARAM_COLOR_MODE "Color Mode"
#define PARAM_BIG_POINTS "Big Points"
#define PARAM_COLOR_MODE_Z_MAX_Z "Red Height"
#define PARAM_COLOR_MODE_Z_MIN_Z "Blue Height"
#define PARAM_ALPHA "Alpha"

#define COLOR_MODE_Z_MAX_Z 15
#define COLOR_MODE_Z_MIN_Z -2
#define COLOR_MODE_Z_DZ 0.1
#define PARAM_MAX_DRAW_Z "Max Draw Z"
#define PARAM_MIN_DRAW_Z "Min Draw Z"

#define PARAM_X         "X    "
#define PARAM_Y         "Y    "
#define PARAM_Z         "Z    "
#define PARAM_ROLL      "Roll "
#define PARAM_PITCH     "Pitch"
#define PARAM_YAW       "Yaw  "

#if 1
#define ERR(...) do { fprintf(stderr, "[%s:%d] ", __FILE__, __LINE__);  \
        fprintf(stderr, __VA_ARGS__); fflush(stderr); } while(0)
#else
#define ERR(...)
#endif

#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG
#define DBG(...) do { fprintf(stdout, __VA_ARGS__); fflush(stdout); } while(0)
#else
#define DBG(...)
#endif

typedef enum _color_mode_t {
  COLOR_MODE_DRAB, COLOR_MODE_Z,
} color_mode_t;

typedef struct {
  BotRenderer renderer;

  BotViewer *viewer;

  /* user parameters */
  BotGtkParamWidget *pw;
  int param_color_mode;
  gboolean param_big_points;
  double param_color_mode_z_max_z;
  double param_color_mode_z_min_z;
  double param_max_draw_z;
  double param_min_draw_z;
  double param_alpha;

  //vertex buffer to draw all points at once
  size_t pointBuffSize;
  double * pointBuffer;

  int numValidPoints;

  BotTrans regTransform;

} RendererPointCloudReg;

static void renderer_point_cloud_reg_destroy(BotRenderer *renderer)
{
  if (!renderer)
    return;

  RendererPointCloudReg *self = (RendererPointCloudReg*) renderer->user;
  if (!self)
    return;

  free(self);
}

static void renderer_point_cloud_reg_draw(BotViewer *viewer, BotRenderer *renderer)
{
  RendererPointCloudReg *self = (RendererPointCloudReg*) renderer->user;
  g_assert(self);

  glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  if (self->param_big_points)
    glPointSize(8.0f);
  else
    glPointSize(2.0f);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glPushMatrix();
  bot_gl_multTrans(&self->regTransform);

  int color_mode = self->param_color_mode;
  double z_norm_scale = 1 / (self->param_color_mode_z_max_z - self->param_color_mode_z_min_z);

  glBegin(GL_POINTS);
  for (int i = 0; i < self->numValidPoints; i++) {
    double * pointV = self->pointBuffer + 3 * i;
    float colorV[4];

    if (pointV[2] < self->param_min_draw_z || pointV[2] > self->param_max_draw_z)
      continue;

    colorV[3] = self->param_alpha;
    switch (color_mode) {
    case COLOR_MODE_Z:
      {
        double z = pointV[2];
        double z_norm = (z - self->param_color_mode_z_min_z) * z_norm_scale;
        float * jetC = bot_color_util_jet(z_norm);
        memcpy(colorV, jetC, 3 * sizeof(float));
      }
      break;
    case COLOR_MODE_DRAB:/* COLOR_MODE_DRAB */
    default:
      {
        colorV[0] = 0.3;
        colorV[1] = 0.3;
        colorV[2] = 0.3;
      }
      break;
    }

    glColor4fv(colorV);
    glVertex3dv(pointV);
  }
  glEnd();

  glPopMatrix();

  glDisable(GL_BLEND);
  glDisable(GL_DEPTH_TEST);

  glPopAttrib();
}

static void on_clear_button(GtkWidget *button, RendererPointCloudReg *self)
{
  if (!self->viewer)
    return;

  self->numValidPoints = 0;
  self->pointBuffSize = 0;
  free(self->pointBuffer);

  bot_viewer_request_redraw(self->viewer);
}

static int load_points_from_file(RendererPointCloudReg *self, FILE *file)
{
//  /* first line: number of columns */
//  fprintf(file, "%%%% %d\n", 10);
//  /* second line: column titles */
//  fprintf(file, "%%%% POINT_X POINT_Y POINT_Z ORIGIN_X ORIGIN_Y ORIGIN_Z"
//      " POINT_STATUS LASER_NUMER SCAN_NUMER POINT_NUMBER\n");

  char * lineptr = NULL;
  size_t lineBufSize = -1;

  //count number of lines in file (number of points in cloud)
  int numPoints;
  while (getline(&lineptr, &lineBufSize, file) > 0) {
    numPoints++;
  }
  rewind(file);
  size_t buffSize = 3 * numPoints * sizeof(double);
  if (self->pointBuffSize < buffSize) {
    self->pointBuffSize = buffSize;
    self->pointBuffer = (double *) realloc(self->pointBuffer, buffSize);
  }

  int cnt = 0;
  self->numValidPoints = 0;
  while (1) {
    int charsInLine = getline(&lineptr, &lineBufSize, file);
    if (charsInLine < 0)
      break;
    if (cnt++ < 2)
      continue;

    double point_xyz[3];
    double origin_xyz[3];
    int point_status;
    int chan_idx;
    int scan_idx;
    int point_num;
    int numParsed = sscanf(lineptr, "%lf %lf %lf %lf %lf %lf %d %d %d %d\n", &point_xyz[0], &point_xyz[1],
        &point_xyz[2],
        &origin_xyz[0], &origin_xyz[1], &origin_xyz[2],
        &point_status, &chan_idx, &scan_idx, &point_num);
    if (numParsed != 10) {
      ERR("count not parse line %d", cnt);
    }
    if (point_status < 3) {
      double * pbuff_xyz = self->pointBuffer + 3 * self->numValidPoints;
      memcpy(pbuff_xyz, point_xyz, 3 * sizeof(double));
      self->numValidPoints++;
    }

  }
  return self->numValidPoints;
}

static void on_load_button(GtkWidget *button, RendererPointCloudReg *self)
{
  GtkWidget *dialog;
  dialog = gtk_file_chooser_dialog_new("Load Data File", NULL, GTK_FILE_CHOOSER_ACTION_OPEN, GTK_STOCK_CANCEL,
      GTK_RESPONSE_CANCEL, GTK_STOCK_OPEN, GTK_RESPONSE_ACCEPT, NULL);
  g_assert(dialog);
  if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
    char *filename;
    filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
    if (NULL != filename) {
      FILE *file = fopen(filename, "r");
      if (NULL != file) {
        int count = load_points_from_file(self, file);

        DBG("Loaded %d points from file '%s'.\n", count, filename);
        fclose(file);
      }
      else {
        ERR("Error: Failed to open file: '%s', error is: '%s'\n",
            filename, g_strerror(errno));
      }
      g_free(filename);
    }
  }
  gtk_widget_destroy(dialog);
  bot_viewer_request_redraw(self->viewer);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererPointCloudReg *self = (RendererPointCloudReg*) user;

  if (!self->viewer)
    return;

  self->param_color_mode = bot_gtk_param_widget_get_enum(self->pw, PARAM_COLOR_MODE);
  self->param_big_points = bot_gtk_param_widget_get_bool(self->pw, PARAM_BIG_POINTS);
  self->param_color_mode_z_max_z = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z);
  self->param_color_mode_z_min_z = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z);
  self->param_max_draw_z = bot_gtk_param_widget_get_double(self->pw, PARAM_MAX_DRAW_Z);
  self->param_min_draw_z = bot_gtk_param_widget_get_double(self->pw, PARAM_MIN_DRAW_Z);
  self->param_alpha = bot_gtk_param_widget_get_double(self->pw, PARAM_ALPHA);

  self->regTransform.trans_vec[0] = bot_gtk_param_widget_get_double(self->pw, PARAM_X);
  self->regTransform.trans_vec[1] = bot_gtk_param_widget_get_double(self->pw, PARAM_Y);
  self->regTransform.trans_vec[2] = bot_gtk_param_widget_get_double(self->pw, PARAM_Z);
  double rpy[3];
  rpy[0] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_ROLL));
  rpy[1] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_PITCH));
  rpy[2] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_YAW));
  bot_roll_pitch_yaw_to_quat(rpy, self->regTransform.rot_quat);

  bot_viewer_request_redraw(self->viewer);
}

static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user)
{
  if (!viewer)
    return;

  RendererPointCloudReg *self = (RendererPointCloudReg*) user;
  bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user)
{
  if (!viewer)
    return;

  RendererPointCloudReg *self = (RendererPointCloudReg*) user;
  bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

static BotRenderer* renderer_point_cloud_reg_new(BotViewer *viewer)
{
  RendererPointCloudReg *self = (RendererPointCloudReg*) calloc(1, sizeof(RendererPointCloudReg));
  g_assert(self);

  self->viewer = viewer;

  BotRenderer *renderer = &self->renderer;
  renderer->draw = renderer_point_cloud_reg_draw;
  renderer->destroy = renderer_point_cloud_reg_destroy;
  renderer->user = self;
  renderer->name = strdup(RENDERER_NAME);
  renderer->enabled = 1;

  self->param_color_mode = COLOR_MODE_Z;
  self->param_big_points = FALSE;
  self->param_color_mode_z_max_z = COLOR_MODE_Z_MAX_Z;
  self->param_color_mode_z_min_z = COLOR_MODE_Z_MIN_Z;
  self->param_max_draw_z = COLOR_MODE_Z_MAX_Z;
  self->param_min_draw_z = COLOR_MODE_Z_MIN_Z;
  self->param_alpha = 1;

  if (viewer) {
    /* setup parameter widget */
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
    renderer->widget = gtk_vbox_new(FALSE, 0);
    gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_enum(self->pw, PARAM_COLOR_MODE, BOT_GTK_PARAM_WIDGET_MENU, self->param_color_mode,
        "Drab", COLOR_MODE_DRAB, "Height", COLOR_MODE_Z,
        NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_DEFAULTS, PARAM_BIG_POINTS, self->param_big_points,
        NULL);

    bot_gtk_param_widget_add_double(self->pw, PARAM_ALPHA, BOT_GTK_PARAM_WIDGET_SLIDER,
        0, 1, .01, self->param_alpha);

    bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z, BOT_GTK_PARAM_WIDGET_SPINBOX,
        COLOR_MODE_Z_MIN_Z, COLOR_MODE_Z_MAX_Z, COLOR_MODE_Z_DZ, self->param_color_mode_z_max_z);
    bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z, BOT_GTK_PARAM_WIDGET_SPINBOX,
        COLOR_MODE_Z_MIN_Z, COLOR_MODE_Z_MAX_Z, COLOR_MODE_Z_DZ, self->param_color_mode_z_min_z);

    bot_gtk_param_widget_add_separator(self->pw, NULL);
    bot_gtk_param_widget_add_double(self->pw, PARAM_MIN_DRAW_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, COLOR_MODE_Z_MIN_Z,
        COLOR_MODE_Z_MAX_Z, COLOR_MODE_Z_DZ, self->param_min_draw_z);
    bot_gtk_param_widget_add_double(self->pw, PARAM_MAX_DRAW_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, COLOR_MODE_Z_MIN_Z,
        COLOR_MODE_Z_MAX_Z, COLOR_MODE_Z_DZ, self->param_max_draw_z);

    bot_gtk_param_widget_add_separator(self->pw, "transform");
    bot_gtk_param_widget_add_double(self->pw, PARAM_X, BOT_GTK_PARAM_WIDGET_SPINBOX, -1000, 1000, 0.001, 0);
    bot_gtk_param_widget_add_double(self->pw, PARAM_Y, BOT_GTK_PARAM_WIDGET_SPINBOX, -1000, 1000, 0.001, 0);
    bot_gtk_param_widget_add_double(self->pw, PARAM_Z, BOT_GTK_PARAM_WIDGET_SPINBOX, -1000, 1000, 0.001, 0);
    bot_gtk_param_widget_add_double(self->pw, PARAM_ROLL, BOT_GTK_PARAM_WIDGET_SPINBOX, -181, 181, 0.1, 0);
    bot_gtk_param_widget_add_double(self->pw, PARAM_PITCH, BOT_GTK_PARAM_WIDGET_SPINBOX, -181, 181, 0.1, 0);
    bot_gtk_param_widget_add_double(self->pw, PARAM_YAW, BOT_GTK_PARAM_WIDGET_SPINBOX, -181, 181, 0.1, 0);

    GtkWidget *clear_button = gtk_button_new_with_label("Clear memory");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_button), "clicked", G_CALLBACK(on_clear_button), self);

    GtkWidget *save_button = gtk_button_new_with_label("Load Points From File");
    gtk_box_pack_start(GTK_BOX(renderer->widget), save_button, FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(save_button), "clicked", G_CALLBACK(on_load_button), self);
    gtk_widget_show_all(renderer->widget);

    /* setup signal callbacks */
    g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
    g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);
  }

  return &self->renderer;
}

void mav_mapping_point_cloud_reg_add_renderer_to_viewer(BotViewer *viewer, int priority)
{
  BotRenderer *renderer = renderer_point_cloud_reg_new(viewer);
  if (viewer && renderer) {
    bot_viewer_add_renderer(viewer, renderer, priority);
  }
}

