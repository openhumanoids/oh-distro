#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <gtk/gtk.h>

#include <lcm/lcm.h>

#include "bot_frames_renderers.h"
#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_vis/viewer.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

//#define RENDERER_NAME "Coord Frames"

#define PARAM_FRAME_SELECT_0 "Frame R"
#define PARAM_FRAME_SELECT_1 "Frame G"
#define PARAM_FRAME_SELECT_2 "Frame B"
#define PARAM_FOLLOW_POS "Follow pos"
#define PARAM_FOLLOW_YAW "yaw"
#define PARAM_SHOW_FRAME "Draw Frame"
#define PARAM_SHOW_SHADOW "Shadow"
#define PARAM_PATH_RENDER_MODE "Render Path"
#define PARAM_MAXPOSES "Max Hist"
#define PARAM_DECIMATE_PATH "Decimate Hist"
//#define PARAM_PATH_COLOR "Path Color"
#define PARAM_FRAME_SIZE "Frame Size"

#define MAX_HIST   10000

typedef struct _RendererFrames {

  BotRenderer renderer;
  BotEventHandler ehandler;
  BotViewer *viewer;
  BotGtkParamWidget *pw;

  BotFrames * frames;

  BotPtrCircular *path_0; // elements: double[3]
  BotPtrCircular *path_1; // elements: double[3]
  BotPtrCircular *path_2; // elements: double[3]

  const char * rootFrame;
  int numFrames;
  char ** frameNames;
  int * frameNums;

  //state for smoothly following a frame
  int smooth_init;
  double smooth_xyzt[4];
  double yaw_offset;
  double yaw_prev;

} RendererFrames;

enum {
  PATH_MODE_NORMAL, PATH_MODE_FLOOR, PATH_MODE_CURTAIN, PATH_MODE_AXES
};

// called by bot_ptr_circular when a path sample needs to be freed.
static void free_path_element(void *user, void *p)
{
  free(p);
}

static void on_find_button(GtkWidget *button, RendererFrames *self)
{
  if (bot_ptr_circular_size(self->path_0) == 0)
    return;

  BotViewHandler *vhandler = self->viewer->view_handler;

  double eye[3];
  double lookat[3];
  double up[3];

  BotTrans * last = (BotTrans *) bot_ptr_circular_index(self->path_0, 0);

  vhandler->get_eye_look(vhandler, eye, lookat, up);
  double diff[3];
  bot_vector_subtract_3d(eye, lookat, diff);

  bot_vector_add_3d(last->trans_vec, diff, eye);

  vhandler->set_look_at(vhandler, eye, last->trans_vec, up);

  bot_viewer_request_redraw(self->viewer);
}

static void destroy_renderer_frames(BotRenderer *super)
{
  RendererFrames *self = (RendererFrames*) super->user;

  bot_ptr_circular_destroy(self->path_0);
  bot_ptr_circular_destroy(self->path_1);
  bot_ptr_circular_destroy(self->path_2);
  free(self);
}

static void draw_axis(BotTrans * axis_to_local, float size, float lineThickness, float opacity)
{
  double axis_to_local_m[16];
  bot_trans_get_mat_4x4(axis_to_local, axis_to_local_m);

  // opengl expects column-major matrices
  double axis_to_local_m_opengl[16];
  bot_matrix_transpose_4x4d(axis_to_local_m, axis_to_local_m_opengl);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_DEPTH_TEST);

  glPushMatrix();
  // rotate and translate the vehicle
  glMultMatrixd(axis_to_local_m_opengl);

  glLineWidth(lineThickness);
  //x-axis
  glBegin(GL_LINES);
  glColor4f(1, 0, 0, opacity);
  glVertex3f(size, 0, 0);
  glVertex3f(0, 0, 0);
  glEnd();

  //y-axis
  glBegin(GL_LINES);
  glColor4f(0, 1, 0, opacity);
  glVertex3f(0, size, 0);
  glVertex3f(0, 0, 0);
  glEnd();

  //z-axis
  glBegin(GL_LINES);
  glColor4f(0, 0, 1, opacity);
  glVertex3f(0, 0, size);
  glVertex3f(0, 0, 0);
  glEnd();

  glPopMatrix();

  glDisable(GL_BLEND);
  glDisable(GL_DEPTH_TEST);

}

static void draw_path(RendererFrames *self, BotTrans *last_coord, BotPtrCircular *path, float jet_value)
{
  int max_draw_poses = bot_gtk_param_widget_get_int(self->pw, PARAM_MAXPOSES);
  if (max_draw_poses == 0)
    return;

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_DEPTH_TEST);

  // float * path_color = bot_color_util_jet(bot_gtk_param_widget_get_double(self->pw, PARAM_PATH_COLOR));
  float * path_color = bot_color_util_jet( jet_value );


  switch (bot_gtk_param_widget_get_enum(self->pw, PARAM_PATH_RENDER_MODE)) {
  case PATH_MODE_AXES:
    {
      draw_axis(last_coord, bot_gtk_param_widget_get_double(self->pw, PARAM_FRAME_SIZE) , 2, .4);
      for (unsigned int i = 0; i < MIN(bot_ptr_circular_size(path), max_draw_poses); i++) {
        BotTrans * t = (BotTrans *) bot_ptr_circular_index(path, i);
        draw_axis(t, bot_gtk_param_widget_get_double(self->pw, PARAM_FRAME_SIZE), 2, .4);
      }
      break;
    }
  case PATH_MODE_FLOOR:
    {
      glColor4f(path_color[0], path_color[1], path_color[2], 0.75);
      glLineWidth(2);
      glBegin(GL_LINE_STRIP);
      glVertex3f(last_coord->trans_vec[0], last_coord->trans_vec[1], 0);
      for (unsigned int i = 0; i < MIN(bot_ptr_circular_size(path), max_draw_poses); i++) {
        BotTrans * t = (BotTrans *) bot_ptr_circular_index(path, i);
        glVertex3f(t->trans_vec[0], t->trans_vec[1], 0);
      }
      glEnd();
      break;
    }
  case PATH_MODE_CURTAIN:
    {
      // draw translucent curtain connecting 3D path to ground.
      glClearStencil(0);
      glClear(GL_STENCIL_BUFFER_BIT);
      glEnable(GL_STENCIL_TEST);
      glStencilFunc(GL_EQUAL, 0, 0xffffffff);
      glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);

      glLineWidth(5);
      glColor4f(0, 0, 1, 0.3);
      glBegin(GL_QUAD_STRIP);
      int nposes = MIN(bot_ptr_circular_size(path), max_draw_poses);
      for (unsigned int i = 0; i < nposes; i++) {
        double *pos = (double *) bot_ptr_circular_index(path, i);
        BotTrans * t = (BotTrans *) bot_ptr_circular_index(path, i);
        glVertex3dv(t->trans_vec);
        glVertex3f(t->trans_vec[0], t->trans_vec[1], 0);
      }
      glEnd();

      glDisable(GL_STENCIL_TEST);
    }
    //don't break, so that we draw the normal line as well
  case PATH_MODE_NORMAL:
    {
      glColor4f(path_color[0], path_color[1], path_color[2], 0.75);
      glLineWidth(2);
      glBegin(GL_LINE_STRIP);
      glVertex3dv(last_coord->trans_vec);
      for (unsigned int i = 0; i < MIN(bot_ptr_circular_size(path), max_draw_poses); i++) {
        BotTrans * t = (BotTrans *) bot_ptr_circular_index(path, i);
        glVertex3dv(t->trans_vec);
      }
      glEnd();
      break;
    }

  }

  glDisable(GL_BLEND);
  glDisable(GL_DEPTH_TEST);

}

static void update_path_hist(RendererFrames *self, BotTrans *frame_to_root, BotPtrCircular *path)
{
  BotTrans t;
  if (bot_ptr_circular_size(path) > 0)
    bot_trans_copy(&t, (BotTrans *) bot_ptr_circular_index(path, 0));
  else
    bot_trans_set_identity(&t);

  double dist = bot_vector_dist_3d(frame_to_root->trans_vec, t.trans_vec);

#if 0
  if (dist > 2.0) {
    // clear the buffer if we jump
    bot_ptr_circular_clear(self->path);
  }
#endif

  if (dist > bot_gtk_param_widget_get_double(self->pw, PARAM_DECIMATE_PATH) || bot_ptr_circular_size(path) == 0) {
    BotTrans *p = (BotTrans*) calloc(1, sizeof(BotTrans));
    bot_trans_copy(p, frame_to_root);
    bot_ptr_circular_add(path, p);
  }

}

static void update_view_follower(RendererFrames *self, BotTrans *frame_to_root)
{
  //smooth the position for following
  double rpy[3];
  bot_quat_to_roll_pitch_yaw(frame_to_root->rot_quat, rpy);
  double yaw = rpy[2];
  //make the yaw contintinuous
  if (yaw - self->yaw_prev >= M_PI) {
    self->yaw_offset -= 2 * M_PI;
  }
  else if (yaw - self->yaw_prev <= -M_PI) {
    self->yaw_offset += 2 * M_PI;
  }
  self->yaw_prev = yaw;
  yaw += self->yaw_offset; //yaw is now continuous

  double xyzt[4] = { frame_to_root->trans_vec[0], frame_to_root->trans_vec[1], frame_to_root->trans_vec[2], yaw };

  if (!self->smooth_init) {
    self->smooth_init = 1;
    memcpy(self->smooth_xyzt, xyzt, 4 * sizeof(double));
  }
  double alpha = .25;
  bot_vector_scale_nd(xyzt, 4, alpha);
  bot_vector_scale_nd(self->smooth_xyzt, 4, 1 - alpha);
  bot_vector_add_nd(self->smooth_xyzt, xyzt, 4, self->smooth_xyzt);
  rpy[0] = 0;
  rpy[1] = 0;
  rpy[2] = self->smooth_xyzt[3];
  double quat[4];
  bot_roll_pitch_yaw_to_quat(rpy, quat);

  BotViewHandler *vhandler = self->viewer->view_handler;
  if (vhandler && vhandler->update_follow_target && vhandler->follow_mode != 0) {
    vhandler->update_follow_target(vhandler, self->smooth_xyzt, quat);
  }
}

static void draw_shadow(BotTrans * frame_to_root)
{
  glColor4f(0, 0, 0, 0.2);
  glPushMatrix();
  glTranslated(frame_to_root->trans_vec[0], frame_to_root->trans_vec[1], 0);
  glTranslated(-0.04, 0.03, 0);
  double rpy[3];
  bot_quat_to_roll_pitch_yaw(frame_to_root->rot_quat, rpy);
  glRotatef(rpy[2] * 180 / M_PI + 45, 0, 0, 1);
  glBegin(GL_QUADS);
  glVertex2f(0.3, -0.3);
  glVertex2f(0.3, 0.3);
  glVertex2f(-0.3, 0.3);
  glVertex2f(-0.3, -0.3);
  glEnd();
  glPopMatrix();
}

static void draw(BotViewer *viewer, BotRenderer *super)
{
  RendererFrames *self = (RendererFrames*) super->user;


  for (int i=0; i <3 ; i++){
    int draw_frame_num;
    const char * draw_frame;
    BotPtrCircular *path;
    if (i==0){
      draw_frame_num = bot_gtk_param_widget_get_enum(self->pw, PARAM_FRAME_SELECT_0);
      draw_frame = bot_gtk_param_widget_get_enum_str(self->pw, PARAM_FRAME_SELECT_0);
      path = self->path_0;
    }else if(i==1){
      draw_frame_num = bot_gtk_param_widget_get_enum(self->pw, PARAM_FRAME_SELECT_1);
      draw_frame = bot_gtk_param_widget_get_enum_str(self->pw, PARAM_FRAME_SELECT_1);
      path = self->path_1;
    }else if(i==2){
      draw_frame_num = bot_gtk_param_widget_get_enum(self->pw, PARAM_FRAME_SELECT_2);
      draw_frame = bot_gtk_param_widget_get_enum_str(self->pw, PARAM_FRAME_SELECT_2);
      path = self->path_2;
    }

    if (draw_frame_num == 0)
      return;

    BotTrans frame_to_root;
    bot_frames_get_trans(self->frames, draw_frame, self->rootFrame, &frame_to_root);

    update_path_hist(self, &frame_to_root, path);

    float jet_value = (float) (2-i) /2;
    draw_path(self, &frame_to_root, path, jet_value);

    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_FRAME)) {
      draw_axis(&frame_to_root, bot_gtk_param_widget_get_double(self->pw, PARAM_FRAME_SIZE), 4, .7); // was 1m, now set in ui
    }

    if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_SHADOW)) {
      draw_shadow(&frame_to_root);
    }

    if (i==0){ // floow the first frame
      if (bot_gtk_param_widget_get_bool(self->pw, PARAM_FOLLOW_POS)) {
        update_view_follower(self, &frame_to_root);
      }
    }
  }


}

static void on_clear_button(GtkWidget *button, RendererFrames *self)
{
  bot_ptr_circular_clear(self->path_0);
  bot_ptr_circular_clear(self->path_1);
  bot_ptr_circular_clear(self->path_2);


  bot_viewer_request_redraw(self->viewer);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererFrames *self = (RendererFrames*) user;
  BotViewer *viewer = self->viewer;

  if (strcmp(name, PARAM_FRAME_SELECT_0) == 0) {
    on_clear_button(NULL, self);
    self->smooth_init = 0; //reset smoothing
  }

  viewer->view_handler->follow_mode = 0;
  if (bot_gtk_param_widget_get_bool(pw, PARAM_FOLLOW_POS))
    viewer->view_handler->follow_mode |= BOT_FOLLOW_POS;
  if (bot_gtk_param_widget_get_bool(pw, PARAM_FOLLOW_YAW))
    viewer->view_handler->follow_mode |= BOT_FOLLOW_YAW;

  bot_viewer_request_redraw(self->viewer);
}

static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererFrames *self = (RendererFrames *) user_data;
  bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, self->renderer.name);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererFrames *self = (RendererFrames *) user_data;
  bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, self->renderer.name);
}

static void frames_update_handler(BotFrames *bot_frames, const char *frame, const char * relative_to, int64_t utime,
    void *user)
{
  RendererFrames *self = (RendererFrames *) user;
  //TODO: handle adding coordinate frames!
  bot_viewer_request_redraw(self->viewer);
}

void bot_frames_add_named_renderer_to_viewer(BotViewer *viewer, int render_priority, BotFrames * frames, const char * renderer_name){

  RendererFrames *self = (RendererFrames*) calloc(1, sizeof(RendererFrames));

  BotRenderer *renderer = &self->renderer;

  renderer->draw = draw;
  renderer->destroy = destroy_renderer_frames;

  renderer->widget = gtk_vbox_new(FALSE, 0);
  renderer->name = strdup(renderer_name);
  renderer->user = self;
  renderer->enabled = 1;

  BotEventHandler *ehandler = &self->ehandler;
  ehandler->name = renderer->name;
  ehandler->enabled = 1;
  ehandler->pick_query = NULL;
  ehandler->key_press = NULL;
  ehandler->hover_query = NULL;
  ehandler->mouse_press = NULL;
  ehandler->mouse_release = NULL;
  ehandler->mouse_motion = NULL;
  ehandler->user = self;

  self->viewer = viewer;
  self->frames = frames;
  bot_frames_add_update_subscriber(self->frames, frames_update_handler, (void *) self);

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  self->path_0 = bot_ptr_circular_new(MAX_HIST, free_path_element, NULL);
  self->path_1 = bot_ptr_circular_new(MAX_HIST, free_path_element, NULL);
  self->path_2 = bot_ptr_circular_new(MAX_HIST, free_path_element, NULL);

  self->rootFrame = bot_frames_get_root_name(self->frames);
  int num_frames = bot_frames_get_num_frames(self->frames);

  char ** frame_names = bot_frames_get_frame_names(self->frames);

  self->numFrames = num_frames + 1; //need space for extra "empty" one
  self->frameNames = calloc(self->numFrames, sizeof(char *));
  self->frameNums = calloc(self->numFrames, sizeof(int));
  self->frameNums[0] = 0;
  self->frameNames[0] = "";
  for (int i = 1; i < self->numFrames; i++) {
    self->frameNums[i] = i;
    self->frameNames[i] = strdup(frame_names[i - 1]);
  }
  g_strfreev(frame_names);
  bot_gtk_param_widget_add_enumv(self->pw, PARAM_FRAME_SELECT_0, BOT_GTK_PARAM_WIDGET_DEFAULTS, 0, self->numFrames,
      (const char **) self->frameNames, self->frameNums);

  bot_gtk_param_widget_add_enumv(self->pw, PARAM_FRAME_SELECT_1, BOT_GTK_PARAM_WIDGET_DEFAULTS, 0, self->numFrames,
      (const char **) self->frameNames, self->frameNums);

  bot_gtk_param_widget_add_enumv(self->pw, PARAM_FRAME_SELECT_2, BOT_GTK_PARAM_WIDGET_DEFAULTS, 0, self->numFrames,
      (const char **) self->frameNames, self->frameNums);

  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);

  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_FOLLOW_POS, 1, PARAM_FOLLOW_YAW, 0,
      NULL);

  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_FRAME, 1, PARAM_SHOW_SHADOW, 0,
      NULL);

  //  self->draw_path_on_floor=0;

  bot_gtk_param_widget_add_enum(self->pw, PARAM_PATH_RENDER_MODE, BOT_GTK_PARAM_WIDGET_DEFAULTS, 0, "Line",
      PATH_MODE_NORMAL, "Line On floor", PATH_MODE_FLOOR, "Curtain", PATH_MODE_CURTAIN, "Axes", PATH_MODE_AXES, NULL);

  //bot_gtk_param_widget_add_double(self->pw, PARAM_PATH_COLOR, BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, .01, .3);
  bot_gtk_param_widget_add_double(self->pw, PARAM_FRAME_SIZE, BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, .01, .3);
  bot_gtk_param_widget_add_double(self->pw, PARAM_DECIMATE_PATH, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 100, .01, .1);

  bot_gtk_param_widget_add_int(self->pw, PARAM_MAXPOSES, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, MAX_HIST, 1, 1000);

  GtkWidget *find_button = gtk_button_new_with_label("Find");
  gtk_box_pack_start(GTK_BOX(renderer->widget), find_button, FALSE, FALSE, 0);
  g_signal_connect(G_OBJECT(find_button), "clicked", G_CALLBACK(on_find_button), self);

  GtkWidget *clear_button = gtk_button_new_with_label("Clear path");
  gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 0);
  g_signal_connect(G_OBJECT(clear_button), "clicked", G_CALLBACK(on_clear_button), self);

  gtk_widget_show_all(renderer->widget);

  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  on_param_widget_changed(self->pw, "", self);

  bot_viewer_add_renderer(viewer, &self->renderer, render_priority);

  g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
  g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);
}

void bot_frames_add_renderer_to_viewer(BotViewer *viewer, int render_priority, BotFrames * frames)
{
  bot_frames_add_named_renderer_to_viewer(viewer,render_priority,frames,"BOT_FRAMES");
}
