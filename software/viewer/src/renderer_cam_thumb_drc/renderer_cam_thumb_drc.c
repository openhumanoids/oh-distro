#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include <gtk/gtk.h>

#define GL_GLEXT_PROTOTYPES 1
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#define BUFFER_OFFSET(i) ((char *)NULL + (i))

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_util.h>

#include <lcmtypes/multisense_images_t.h>

#include <image_utils/jpeg.h>
#include <image_utils/pixels.h>
#include <geom_utils/geometry.h>
#include "renderer_cam_thumb_drc.h"

#define RENDERER_NAME "Camera"

#define PARAM_AT_GROUND_SCALE "At Camera Scale"

#define PARAM_COLOR_ALPHA "Alpha"
#define PARAM_RENDER_IN "Show"
#define PARAM_UPRIGHT "Upright"
#define PARAM_IMAGE_SCALE "Scale"

#define MAX_GROUND_PROJECTION_DISTANCE 120
#define MAX_GROUND_PROJECTION_DISTANCE_SQ (MAX_GROUND_PROJECTION_DISTANCE*MAX_GROUND_PROJECTION_DISTANCE)

typedef struct _RendererCamThumb RendererCamThumb;

struct _RendererCamThumb {
  BotRenderer renderer;

  BotFrames *bot_frames;

  BotGtkParamWidget *pw;

  GHashTable *cam_handlers;
  BotParam *param;

  lcm_t *lc;
  BotViewer *viewer;
  
  // opacity
  float alpha;
  // scale of at camera distance:
  float at_camera_scale;
  // has the scale changed - need new dl (boolean)
  int need_new_camera_dl;

  // height of ground plane - provided via POSE_GROUND message
  // Otherwise = 0. this could be inferred by any method e.g. lowest foot or from perception
  float ground_height;  
};

typedef struct _ImageVertex ImageVertex;
struct _ImageVertex {
  float tx;
  float ty;
  float vx;
  float vy;
  float vz;
  int ground_candidate;
};

typedef struct _cam_renderer {
  char *channel;
  BotGtkGlDrawingArea *gl_area;
  bot_core_image_t *last_image;
  BotGlTexture *texture;
  RendererCamThumb *renderer;
  BotGtkParamWidget *pw;
  GtkWidget *expander;
  BotCamTrans *camtrans;
  char * coord_frame;
  //GtkAspectFrame *aspect_frame;
  uint8_t *uncompresed_buffer;
  int uncompressed_buffer_size;
  int width, height;
  int is_uploaded;

  //    GLuint vbo;
  GLuint at_camera_dl;
  int img_nvertices;
  ImageVertex *vertices;
  int *vert_indices;
  int n_vert_indices;

  point3d_t *pixel_rays_local;
  point2d_t *ground_projections_local;

  int msg_received;
  int render_place;
  int expanded;
  int upright;
  int64_t upright_update_time;
  double upright_theta;
  double image_scale;

//    CamrendVisualFeatures *feature_renderer;
} cam_renderer_t;

enum {
  RENDER_IN_WIDGET,
  RENDER_IN_TOP_RIGHT,
  RENDER_IN_TOP_CENTER,
  RENDER_IN_TOP_LEFT,
  RENDER_IN_BOTTOM_RIGHT,
  RENDER_IN_BOTTOM_CENTER,
  RENDER_IN_BOTTOM_LEFT,
  RENDER_IN_TOP_LEFT_LARGE,
  RENDER_IN_TOP_RIGHT_LARGE,
  RENDER_IN_TOP,
  RENDER_AT_CAMERA,
  RENDER_ON_GROUND,
};

static inline void _check_gl_errors(const char *label)
{
  GLenum errCode = glGetError();
  const GLubyte *errStr;

  while (errCode != GL_NO_ERROR) {
    errStr = gluErrorString(errCode);
    fprintf(stderr, "OpenGL Error (%s)\n", label);
    fprintf(stderr, "%s", (char*) errStr);
    fprintf(stderr, "\n");
    errCode = glGetError();
  }
}

static void cam_renderer_destroy(cam_renderer_t *cr)
{
  if (cr->last_image) {
    bot_core_image_t_destroy(cr->last_image);
  }
  if (cr->camtrans) {
    bot_camtrans_destroy(cr->camtrans);
  }
  if (cr->coord_frame)
    free(cr->coord_frame);
  // TODO cleanly release display list
  free(cr->vertices);
  free(cr->vert_indices);
  free(cr->ground_projections_local);
  free(cr->pixel_rays_local);

  // TODO
  //    if (cr->texture) {
  //        glutil_texture_free (cr->texture);
  //    }
  //    if (cr->feature_renderer)
  //        camrend_visual_features_destroy (cr->feature_renderer);
  if (cr->uncompresed_buffer) {
    free(cr->uncompresed_buffer);
    cr->uncompresed_buffer = NULL;
  }
  free(cr->channel);
  // specifically do not delete the gl_area
  free(cr);
}

static void on_pose_ground(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_pose_t *msg, void *user_data);
static void on_image(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_image_t *msg, void *user_data);
static void on_images(const lcm_recv_buf_t *rbuf, const char *channel, const multisense_images_t *msg, void *user_data);
static void cam_renderer_draw(cam_renderer_t *cr);
static int cam_renderer_prepare_texture(cam_renderer_t *cr);

static void _draw_thumbs_at_cameras(RendererCamThumb *self)
{
  
  // transform into body frame
  GList *crlist = bot_g_hash_table_get_vals(self->cam_handlers);
  for (GList *criter = crlist; criter; criter = criter->next) {
    cam_renderer_t *cr = (cam_renderer_t*) criter->data;

    if (!cr->last_image || !cr->camtrans)
      continue;

    int rmode = bot_gtk_param_widget_get_enum(cr->pw, PARAM_RENDER_IN);
    if (rmode != RENDER_AT_CAMERA && rmode != RENDER_ON_GROUND)
      continue;

    if (0 != cam_renderer_prepare_texture(cr))
      continue;

    // load the texture
    GLuint texname = bot_gl_texture_get_texname(cr->texture);
    GLenum textarget = bot_gl_texture_get_target(cr->texture);

    glEnable(textarget);
    glBindTexture(textarget, texname);

    float texcoord_scale_x = 1;
    float texcoord_scale_y = 1;
    if (textarget == GL_TEXTURE_2D) {
      texcoord_scale_x = 1 / (float) cr->width;
      texcoord_scale_y = 1 / (float) cr->height;
    }

    // was pre jan2013:
    // disable the depth mask while drawing these images
    //glDepthMask(GL_FALSE);
    //    glColor3f(1, 1, 1);

    // added pre jan2013: for variable opacity
    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    

    glColor4f(1, 1, 1, self->alpha);

    // draw
    if (rmode == RENDER_AT_CAMERA) {
      BotTrans cam_to_local;
      bot_frames_get_trans(self->bot_frames, cr->coord_frame, bot_frames_get_root_name(self->bot_frames), &cam_to_local);

      double c2l_mat[16], c2l_mat_gl[16];
      bot_trans_get_mat_4x4(&cam_to_local, c2l_mat);
      bot_matrix_transpose_4x4d(c2l_mat, c2l_mat_gl);

      // transform into body frame
      glPushMatrix();
      glMultMatrixd(c2l_mat_gl);

      if (cr->at_camera_dl && !self->need_new_camera_dl ) {
        glCallList(cr->at_camera_dl);
      }
      else {
        // compile a display list if needed
        cr->at_camera_dl = glGenLists(1);
        glNewList(cr->at_camera_dl, GL_COMPILE);

        glBegin(GL_QUADS);
        for (int i = 0; i < cr->n_vert_indices; i++) {
          ImageVertex *v = &cr->vertices[cr->vert_indices[i]];
          glTexCoord2f(v->tx, v->ty);
          // glTexCoord3f(v->tx, v->ty, 0.5f);
          // glVertex3f(v->vx, v->vy, v->vz);
	  glVertex3f(self->at_camera_scale *v->vx, self->at_camera_scale* v->vy, self->at_camera_scale*v->vz);
        }
        glEnd();

        glEndList();
	
      }

      glPopMatrix();
    }
    else if (rmode == RENDER_ON_GROUND) {
      BotTrans cam_to_local;
      bot_frames_get_trans_with_utime(self->bot_frames, cr->coord_frame, bot_frames_get_root_name(self->bot_frames),
          cr->last_image->utime, &cam_to_local);
      // Project the camera onto a plane estimated from the foot/bottom of the robot
      cam_to_local.trans_vec[2] = cam_to_local.trans_vec[2] - self->ground_height;

      // project image onto the ground plane
      for (int i = 0; i < cr->img_nvertices; i++) {
        ImageVertex *v = &cr->vertices[i];
        if (!v->ground_candidate)
          continue;

        cr->pixel_rays_local[i].x = v->vx;
        cr->pixel_rays_local[i].y = v->vy;
        cr->pixel_rays_local[i].z = v->vz;
        double v_cam[3] = { v->vx, v->vy, v->vz };

        bot_trans_rotate_vec(&cam_to_local, v_cam, point3d_as_array(&cr->pixel_rays_local[i]));

        if (0 != geom_ray_z_plane_intersect_3d(POINT3D(cam_to_local.trans_vec), &cr->pixel_rays_local[i], 0,
        //                if(0 != geom_ray_z_plane_intersect_3d(POINT3D(cam_pos_body),
            //                            &cr->pixel_rays_local[i], 0,
            &cr->ground_projections_local[i])) {
          cr->ground_projections_local[i].x = NAN;
          cr->ground_projections_local[i].y = NAN;
          continue;
        }

        double dist_sq = geom_point_point_distance_squared_2d(&cr->ground_projections_local[i],
            POINT2D(cam_to_local.trans_vec));
        //                            POINT2D(cam_pos_body));

        if (dist_sq > MAX_GROUND_PROJECTION_DISTANCE_SQ) {
          cr->ground_projections_local[i].x = NAN;
          cr->ground_projections_local[i].y = NAN;
        }
      }

      // draw the texels that have a reasonable chance of actually being on the ground plane
      glBegin(GL_QUADS);
      for (int i = 0; i < cr->n_vert_indices; i += 4) {
        ImageVertex *v0 = &cr->vertices[cr->vert_indices[i + 0]];
        ImageVertex *v1 = &cr->vertices[cr->vert_indices[i + 1]];
        ImageVertex *v2 = &cr->vertices[cr->vert_indices[i + 2]];
        ImageVertex *v3 = &cr->vertices[cr->vert_indices[i + 3]];

        point2d_t *p0 = &cr->ground_projections_local[cr->vert_indices[i + 0]];
        point2d_t *p1 = &cr->ground_projections_local[cr->vert_indices[i + 1]];
        point2d_t *p2 = &cr->ground_projections_local[cr->vert_indices[i + 2]];
        point2d_t *p3 = &cr->ground_projections_local[cr->vert_indices[i + 3]];

        if (!(v0->ground_candidate && v1->ground_candidate && v2->ground_candidate && v3->ground_candidate))
          continue;

        if (isnan(p0->x) || isnan(p1->x) || isnan(p2->x) || isnan(p3->x))
          continue;

        glTexCoord2f(v0->tx, v0->ty);
        glVertex3f(p0->x, p0->y, self->ground_height);
        glTexCoord2f(v1->tx, v1->ty);
        glVertex3f(p1->x, p1->y, self->ground_height);
        glTexCoord2f(v2->tx, v2->ty);
        glVertex3f(p2->x, p2->y, self->ground_height);
        glTexCoord2f(v3->tx, v3->ty);
        glVertex3f(p3->x, p3->y, self->ground_height);
      }
      glEnd();
    }
    glDepthMask(GL_TRUE);

    glBindTexture(textarget, 0);
    glDisable(textarget);
  }
  
  // Set the new scale flag to false
  self->need_new_camera_dl=0;
}

static void cam_thumb_draw(BotViewer *viewer, BotRenderer *renderer)
{
  RendererCamThumb *self = (RendererCamThumb*) renderer->user;

  _draw_thumbs_at_cameras(self);

  // transform into window coordinates, where <0, 0> is the top left corner
  // of the window and <viewport[2], viewport[3]> is the bottom right corner
  // of the window
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, viewport[2], 0, viewport[3]);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslatef(0, viewport[3], 0);
  glScalef(1, -1, 1);

  glEnable(GL_DEPTH_TEST);

  double vp_width = viewport[2] - viewport[0];
  double vp_height = viewport[3] - viewport[1];

  GList *crlist = bot_g_hash_table_get_vals(self->cam_handlers);
  for (GList *criter = crlist; criter; criter = criter->next) {
    cam_renderer_t *cr = (cam_renderer_t*) criter->data;

    if (!cr->last_image)
      continue;
    double aspect = cr->last_image->width / (double) cr->last_image->height;

    double thumb_width, thumb_height;
    double scale = cr->image_scale;
    if ((vp_width*scale) / aspect > vp_height*scale) {
      thumb_height = vp_height*scale;
      thumb_width = thumb_height * aspect;
    }
    else {
      thumb_width = vp_width*scale;
      thumb_height = thumb_width / aspect;
    }

    int rmode = bot_gtk_param_widget_get_enum(cr->pw, PARAM_RENDER_IN);
    if (rmode == RENDER_IN_WIDGET || rmode == RENDER_AT_CAMERA || rmode == RENDER_ON_GROUND)
      continue;

    point2d_t p1 = { viewport[0], viewport[1] };

    switch (rmode) {
    case RENDER_IN_BOTTOM_RIGHT:
      p1.x = vp_width - thumb_width;
      p1.y = vp_height - thumb_height;
      break;
    case RENDER_IN_BOTTOM_CENTER:
      p1.x = vp_width/2*(1-scale);
      p1.y = vp_height - thumb_height;
      break;
    case RENDER_IN_BOTTOM_LEFT:
      p1.x = 0;
      p1.y = vp_height - thumb_height;
      break;
    case RENDER_IN_TOP_LEFT:
      p1.x = 0;
      p1.y = 0;
      break;
    case RENDER_IN_TOP_CENTER:
      p1.x = vp_width/2*(1-scale);
      p1.y = 0;
      break;
    case RENDER_IN_TOP_RIGHT:
      p1.x = vp_width - thumb_width;
      p1.y = 0;
      break;
    case RENDER_IN_TOP_LEFT_LARGE:
      if ((vp_width / 2) / aspect > vp_height / 2) {
        thumb_height = vp_height / 2;
        thumb_width = thumb_height * aspect;
      }
      else {
        thumb_width = vp_width / 2;
        thumb_height = thumb_width / aspect;
      }
      
      p1.x = 0;
      p1.y = 0;
      break;
    case RENDER_IN_TOP_RIGHT_LARGE:
      if ((vp_width / 3) / aspect > vp_height / 3) {
        thumb_height = vp_height / 3;
        thumb_width = thumb_height * aspect;
      }
      else {
        thumb_width = vp_width / 3;
        thumb_height = thumb_width / aspect;
      }
      
      p1.x = vp_width - thumb_width;
      p1.y = 0;
      break;
    case RENDER_IN_TOP:
      thumb_width = vp_width;
      thumb_height = thumb_width / aspect;
      p1.x = 0;
      p1.y = 0;
    default:
      break;
    }

    glPushMatrix();
    glTranslatef(p1.x, p1.y, 1);

    if (cr->last_image->utime != cr->upright_update_time) {
      BotTrans local_to_cam;
      if (bot_frames_get_trans_with_utime(cr->renderer->bot_frames, bot_frames_get_root_name(cr->renderer->bot_frames), cr->coord_frame, cr->last_image->utime, &local_to_cam) != 0) {
        double local_to_cam_mat[16];
        bot_trans_get_mat_4x4(&local_to_cam, local_to_cam_mat);
        cr->upright_theta = atan2(local_to_cam_mat[6], local_to_cam_mat[2]);
        cr->upright_theta = -cr->upright_theta*180/M_PI - 90;
        cr->upright_update_time = cr->last_image->utime;
      }
    }
    if ((cr->upright) && (cr->upright_update_time == cr->last_image->utime)) {
      glTranslatef(thumb_width/2.0f,thumb_height/2.0f,0);
      glRotatef(cr->upright_theta,0,0,1);
      glTranslatef(-thumb_width/2.0f,-thumb_height/2.0f,0);
    }

    glScalef(thumb_width, thumb_height, 1);

    cam_renderer_draw(cr);
    glPopMatrix();
  }
  g_list_free(crlist);

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererCamThumb *self = user_data;

  GError *gerr = NULL;
  char **keys = g_key_file_get_keys(keyfile, RENDERER_NAME, NULL, &gerr);
  if (gerr) {
    g_error_free(gerr);
    return;
  }
  for (int i = 0; keys[i]; i++) {
    char *key = keys[i];
    cam_renderer_t *cr = g_hash_table_lookup(self->cam_handlers, key);
    if (!cr) {
      cr = (cam_renderer_t*) calloc(1, sizeof(cam_renderer_t));
      cr->channel = strdup(key);
      cr->renderer = self;
      g_hash_table_replace(self->cam_handlers, cr->channel, cr);
    }
    char *val = g_key_file_get_string(keyfile, RENDERER_NAME, key, NULL);
    cr->render_place = 0;
    cr->expanded = 0;
    cr->upright = 0;
    cr->image_scale = 0.3;
    cr->upright_theta = 0;
    cr->upright_update_time = -1;
    if (val) {
      sscanf(val, "%d %d %d", &cr->render_place, &cr->expanded, &cr->upright);
    }
    g_free(val);
  }
  g_strfreev(keys);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererCamThumb *self = user_data;
  GList *keys = bot_g_hash_table_get_keys(self->cam_handlers);

  for (GList *kiter = keys; kiter; kiter = kiter->next) {
    char *key = (char*) kiter->data;
    cam_renderer_t *cr = g_hash_table_lookup(self->cam_handlers, key);

    char str[80];
    sprintf(str, "%d %d %d", cr->render_place, cr->expanded, cr->upright);
    g_key_file_set_string(keyfile, RENDERER_NAME, key, str);
  }
  g_list_free(keys);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererCamThumb *self = (RendererCamThumb*) user;
  if(! strcmp(name, PARAM_COLOR_ALPHA)) {
    self->alpha = (float) bot_gtk_param_widget_get_double(pw, PARAM_COLOR_ALPHA);
    bot_viewer_request_redraw(self->viewer);
  }
  if(! strcmp(name, PARAM_AT_GROUND_SCALE)) {
    self->need_new_camera_dl = 1;
    self->at_camera_scale = (float) bot_gtk_param_widget_get_double(pw, PARAM_AT_GROUND_SCALE);
    bot_viewer_request_redraw(self->viewer);
  }
}


static void cam_thumb_free(BotRenderer *renderer)
{
  RendererCamThumb *self = (RendererCamThumb*) renderer;

  g_hash_table_destroy(self->cam_handlers);
  free(renderer);
}

static BotRenderer *_new(BotViewer *viewer, lcm_t * lcm, BotParam * param, BotFrames * frames)
{
  RendererCamThumb *self = (RendererCamThumb*) calloc(1, sizeof(RendererCamThumb));
  self->viewer = viewer;
  self->renderer.draw = cam_thumb_draw;
  self->renderer.destroy = cam_thumb_free;
  self->renderer.name = RENDERER_NAME;
  self->renderer.user = self;
  self->renderer.enabled = 1;

  self->bot_frames = frames;
  self->lc = lcm;

  self->renderer.widget = bot_gtk_param_widget_new();//gtk_vbox_new(FALSE, 0);
  gtk_widget_show(self->renderer.widget);

  self->cam_handlers = g_hash_table_new_full(g_str_hash, g_str_equal, NULL, (GDestroyNotify) cam_renderer_destroy);
  self->param = param;
  
  self->alpha = 1.0;
  self->need_new_camera_dl =0; // bool false
  self->at_camera_scale= 1.0;
  self->ground_height = 0.0;
  
  self->pw = BOT_GTK_PARAM_WIDGET(self->renderer.widget);
  bot_gtk_param_widget_add_double (self->pw, PARAM_AT_GROUND_SCALE, BOT_GTK_PARAM_WIDGET_SLIDER, 0.1, 2, 0.1, 1);
  bot_gtk_param_widget_add_double (self->pw, PARAM_COLOR_ALPHA, BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, 0.001, 1);

  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  
  char **cam_names = bot_param_get_all_camera_names(self->param);

  //failure if there are no cam names
  if (cam_names == NULL) {
    free(self);
    fprintf(stderr, "error, no camera names detected, won't add %s renderer\n", RENDERER_NAME);
    return NULL;
  }

  for (int i = 0; cam_names[i]; i++) {
    char * channel = bot_param_get_camera_lcm_channel(self->param, cam_names[i]);
    if (channel != NULL) {

      char key[2048];
      sprintf(key, "cameras.%s.type",  cam_names[i]);
      char *type = NULL;
      int typical_image = 1;
      if (0 == bot_param_get_str(self->param, key, &type)) {
        //fprintf(stdout, "type field found %s\n",type);
        if (strcmp (type, "stereo") == 0){
          //fprintf(stdout, "stereo found type %s\n",type);
          typical_image = 0;
        }
      }
      
      if (typical_image){
        bot_core_image_t_subscribe(self->lc, channel, on_image, self);
      }else{
        //fprintf(stdout, "cam renderer ignoring stereo images on [%s]\n", channel);
      }
      free(channel);
    }
  }
  g_strfreev(cam_names);
  bot_core_pose_t_subscribe(self->lc, "POSE_GROUND", on_pose_ground, self);

  multisense_images_t_subscribe(self->lc, "CAMERA", on_images, self);
 
  g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
  g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);

  return &self->renderer;
}

void add_cam_thumb_drc_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t * lcm, BotParam * param,
    BotFrames * frames)
{
  BotRenderer * renderer =_new(viewer, lcm, param, frames);
  if (renderer != NULL)
    bot_viewer_add_renderer(viewer, renderer, render_priority);
}

static int cam_renderer_prepare_texture(cam_renderer_t *cr)
{
  if (!cr->renderer->renderer.enabled)
    return -1;
  if (!cr->last_image)
    return -1;

  // create the texture object if necessary
  if (!cr->texture) {
    cr->texture = bot_gl_texture_new(cr->last_image->width, cr->last_image->height, cr->last_image->width * 3
        * cr->last_image->height);
  }

  // upload the texture to the graphics card if necessary
  if (!cr->is_uploaded) {
    int stride = 0;
    GLenum gl_format;
    uint8_t *tex_src = NULL;

    if (cr->last_image->pixelformat == 0 || cr->last_image->pixelformat == PIXEL_FORMAT_GRAY
        || cr->last_image->pixelformat == PIXEL_FORMAT_BAYER_BGGR || cr->last_image->pixelformat
        == PIXEL_FORMAT_BAYER_RGGB || cr->last_image->pixelformat == PIXEL_FORMAT_BAYER_GRBG
        || cr->last_image->pixelformat == PIXEL_FORMAT_BAYER_GBRG) {

      stride = cr->last_image->width;
      gl_format = GL_LUMINANCE;
      tex_src = cr->last_image->data;
    } else if (cr->last_image->pixelformat == PIXEL_FORMAT_RGB) {
      stride = cr->last_image->row_stride;
      gl_format = GL_RGB;
      tex_src = cr->last_image->data;
    }
    else if (cr->last_image->pixelformat == PIXEL_FORMAT_MJPEG) {
      bot_core_image_t * msg = cr->last_image;

      // might need to JPEG decompress...
      stride = cr->last_image->width * 3;
      int buf_size = msg->height * stride;
      if (cr->uncompressed_buffer_size < buf_size) {
        cr->uncompresed_buffer = realloc(cr->uncompresed_buffer, buf_size);
        cr->uncompressed_buffer_size = buf_size;
      }
      jpeg_decompress_8u_rgb(msg->data, msg->size, cr->uncompresed_buffer, msg->width, msg->height, stride);

      gl_format = GL_RGB;
      tex_src = cr->uncompresed_buffer;
    }
    else {
      return -1;
    }
    bot_gl_texture_upload(cr->texture, gl_format, GL_UNSIGNED_BYTE, stride, tex_src);
    cr->is_uploaded = 1;
  }

  if (!cr->vertices && cr->camtrans) {
    int xstep = 4;
    int ystep = 12;
    int ncols = cr->width / xstep + 1;
    int nrows = cr->height / ystep + 1;
    cr->img_nvertices = ncols * nrows;
    int img_data_size = cr->img_nvertices * sizeof(ImageVertex);
    cr->vertices = (ImageVertex*) malloc(img_data_size);

    cr->n_vert_indices = (ncols - 1) * (nrows - 1) * 4;

    //cr->n_vert_indices = cr->width / xstep * cr->height / ystep * 4;
    cr->vert_indices = (int*) malloc(cr->n_vert_indices * sizeof(int));
    int vi_count = 0;

    // allocate workspace for projecting the image onto the ground plane
    cr->pixel_rays_local = (point3d_t*) malloc(cr->img_nvertices * sizeof(point3d_t));
    cr->ground_projections_local = (point2d_t*) malloc(cr->img_nvertices * sizeof(point2d_t));

    // precompute undistorted coordinates
    GLenum textarget = bot_gl_texture_get_target(cr->texture);

    float texcoord_scale_x = 1;
    float texcoord_scale_y = 1;
    if (textarget == GL_TEXTURE_2D) {
      texcoord_scale_x = 1 / (float) cr->width;
      texcoord_scale_y = 1 / (float) cr->height;
    }

    ImageVertex *v = cr->vertices;

    BotTrans cam_to_body;
    bot_frames_get_trans(cr->renderer->bot_frames, cr->coord_frame, "body", &cam_to_body);
    double *cam_pos_body = cam_to_body.trans_vec;

    for (int row = 0; row < nrows; row++) {
      int y = row * ystep;
      for (int col = 0; col < ncols; col++) {
        int x = col * xstep;
        double ray_cam[3], ray_body[3];

        bot_camtrans_unproject_pixel(cr->camtrans, x, y, ray_cam);

        bot_vector_normalize_3d(ray_cam);
        bot_trans_rotate_vec(&cam_to_body, ray_cam, ray_body);

        v->tx = x * texcoord_scale_x;
        v->ty = y * texcoord_scale_y;
        v->vx = ray_cam[0];
        v->vy = ray_cam[1];
        v->vz = ray_cam[2];

        v->ground_candidate = 1; //3D vehicles -> all rays are ground candidates...
        //        if (ray_body[2] < 0) {
        //          point2d_t isect_pt;
        //          if (0 == geom_ray_z_plane_intersect_3d(POINT3D(cam_pos_body), POINT3D(ray_body), 0, &isect_pt)) {
        //
        //            double dist = geom_point_point_distance_squared_2d(&isect_pt, POINT2D(cam_pos_body));
        //
        //            if (dist < MAX_GROUND_PROJECTION_DISTANCE_SQ) {
        //              v->ground_candidate = 1;
        //            }
        //          }
        //        }

        v++;

        if (row < nrows - 1 && col < ncols - 1) {
          cr->vert_indices[vi_count + 0] = row * ncols + col;
          cr->vert_indices[vi_count + 1] = row * ncols + col + 1;
          cr->vert_indices[vi_count + 2] = (row + 1) * ncols + col + 1;
          cr->vert_indices[vi_count + 3] = (row + 1) * ncols + col;

          for (int i = 0; i < 4; i++)
            assert(cr->vert_indices[vi_count + i] < cr->img_nvertices);

          vi_count += 4;
        }
        assert(vi_count <= cr->n_vert_indices);
      }
    }
    assert(vi_count == cr->n_vert_indices);
  }

  return 0;
}

static void cam_renderer_draw(cam_renderer_t *cr)
{
  cam_renderer_prepare_texture(cr);
  glColor3f(1, 1, 1);
  bot_gl_texture_draw(cr->texture);
}

static gboolean on_gl_area_expose(GtkWidget * widget, GdkEventExpose * event, void* user_data)
{
  cam_renderer_t *cr = (cam_renderer_t*) user_data;

  bot_gtk_gl_drawing_area_set_context(cr->gl_area);

  glClearColor(0.0, 0.0, 0.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glOrtho(0, 1, 1, 0, -1, 1);
  glMatrixMode(GL_MODELVIEW);

  cam_renderer_draw(cr);

  bot_gtk_gl_drawing_area_swap_buffers(cr->gl_area);

  bot_gtk_gl_drawing_area_set_context(cr->renderer->viewer->gl_area);
  return TRUE;
}

static void on_gl_area_size(GtkWidget * widget, GtkAllocation * alloc, cam_renderer_t * cr)
{
  gtk_widget_set_size_request(widget, -1, alloc->width * cr->height / cr->width);
}

static void on_expander_expanded(GtkExpander *expander, GParamSpec *pspec, void *user_data)
{
  cam_renderer_t *cr = user_data;
  cr->expanded = gtk_expander_get_expanded(expander);
}

static void on_cam_renderer_param_widget_changed(BotGtkParamWidget *pw, const char *param, void *user_data)
{
  cam_renderer_t *cr = (cam_renderer_t*) user_data;

  // delete the old texture object if it exists.  make sure that we've
  // selected the correct OpenGL context
  if (cr->texture) {
    if (cr->render_place == RENDER_IN_WIDGET) {
      bot_gtk_gl_drawing_area_set_context(cr->gl_area);
    }
    else {
      bot_gtk_gl_drawing_area_set_context(cr->renderer->viewer->gl_area);
    }

    bot_gl_texture_free(cr->texture);
    cr->texture = NULL;
  }

  cr->upright = bot_gtk_param_widget_get_bool(pw, PARAM_UPRIGHT);
  cr->image_scale = bot_gtk_param_widget_get_double(pw, PARAM_IMAGE_SCALE);

  cr->render_place = bot_gtk_param_widget_get_enum(pw, PARAM_RENDER_IN);
  if (cr->render_place == RENDER_IN_WIDGET) {
    gtk_widget_show(GTK_WIDGET(cr->gl_area));
  }
  else {
    gtk_widget_hide(GTK_WIDGET(cr->gl_area));
  }

  cr->is_uploaded = 0;
  bot_viewer_request_redraw(cr->renderer->viewer);
}

static void on_pose_ground(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_pose_t *msg, void *user_data)
{
  RendererCamThumb *self = (RendererCamThumb*) user_data;
  self->ground_height =(float) msg->pos[2];
}

static void on_images(const lcm_recv_buf_t *rbuf, const char *channel, const multisense_images_t *msg, void *user_data)
{
  for (int i = 0; i < msg->n_images; ++i) {
    if (msg->image_types[i] == MULTISENSE_IMAGES_T_LEFT) {
      char new_channel[256];
      sprintf(new_channel, "%s_LEFT", channel);
      return on_image(rbuf, new_channel, &(msg->images[0]), user_data);
    }
  }
}

static void on_image(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_image_t *msg, void *user_data)
{
  RendererCamThumb *self = (RendererCamThumb*) user_data;

  cam_renderer_t *cr = g_hash_table_lookup(self->cam_handlers, channel);
  if (!cr) {
    cr = (cam_renderer_t*) calloc(1, sizeof(cam_renderer_t));
    cr->renderer = self;
    cr->render_place = 0;
    cr->channel = strdup(channel);
    cr->at_camera_dl = 0;
    g_hash_table_replace(self->cam_handlers, cr->channel, cr);
  }

  if (!cr->msg_received) {
    cr->gl_area = BOT_GTK_GL_DRAWING_AREA (bot_gtk_gl_drawing_area_new (FALSE));

    cr->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
    bot_gtk_param_widget_add_enum(cr->pw, PARAM_RENDER_IN, 0, cr->render_place, "Here", RENDER_IN_WIDGET, 
        "Top Left", RENDER_IN_TOP_LEFT, "Top Cent.", RENDER_IN_TOP_CENTER, 
        "Top Right", RENDER_IN_TOP_RIGHT, "Bot. Left", RENDER_IN_BOTTOM_LEFT, 
        "Bot. Cent.", RENDER_IN_BOTTOM_CENTER, "Bot. Right", RENDER_IN_BOTTOM_RIGHT,
        "Top L Lrg", RENDER_IN_TOP_LEFT_LARGE,
        "Top Full", RENDER_IN_TOP,
        "At Camera", RENDER_AT_CAMERA, "Ground", RENDER_ON_GROUND, NULL);
    bot_gtk_param_widget_add_booleans(cr->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_UPRIGHT, 0, NULL);
    bot_gtk_param_widget_add_double(cr->pw, PARAM_IMAGE_SCALE,
        BOT_GTK_PARAM_WIDGET_SLIDER, 0.1, 1.0, 0.01, 0.25);
    cr->expander = gtk_expander_new(channel);
    gtk_box_pack_start(GTK_BOX(self->renderer.widget), cr->expander, TRUE, TRUE, 0);
    GtkWidget *vbox = gtk_vbox_new(FALSE, 0);
    gtk_container_add(GTK_CONTAINER(cr->expander), vbox);

    gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(cr->pw), TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox), GTK_WIDGET(cr->gl_area), TRUE, TRUE, 0);

    g_signal_connect(G_OBJECT(cr->gl_area), "size-allocate", G_CALLBACK(on_gl_area_size), cr);
    cr->width = msg->width;
    cr->height = msg->height;

    gtk_widget_show_all(GTK_WIDGET(cr->expander));
    gtk_expander_set_expanded(GTK_EXPANDER(cr->expander), cr->expanded);
    bot_gtk_param_widget_set_bool(cr->pw, PARAM_UPRIGHT, cr->upright);
    bot_gtk_param_widget_set_double(cr->pw, PARAM_IMAGE_SCALE, cr->image_scale);

    if (cr->render_place == RENDER_IN_WIDGET) {
      gtk_widget_show(GTK_WIDGET(cr->gl_area));
    }
    else {
      gtk_widget_hide(GTK_WIDGET(cr->gl_area));
    }

    g_signal_connect(G_OBJECT(cr->pw), "changed", G_CALLBACK(on_cam_renderer_param_widget_changed), cr);
    g_signal_connect(G_OBJECT(cr->gl_area), "expose-event", G_CALLBACK(on_gl_area_expose), cr);

    g_signal_connect(G_OBJECT(cr->expander), "notify::expanded", G_CALLBACK(on_expander_expanded), cr);

    cr->texture = NULL;
    cr->last_image = NULL;
    cr->renderer = self;
    cr->uncompressed_buffer_size = msg->width * msg->height * 3;
    cr->uncompresed_buffer = (uint8_t*) malloc(cr->uncompressed_buffer_size);

    char * cam_name = bot_param_get_camera_name_from_lcm_channel(self->param, channel);
    if (cam_name != NULL) {
      cr->camtrans = bot_param_get_new_camtrans(self->param, cam_name);
      cr->coord_frame = bot_param_get_camera_coord_frame(self->param, cam_name);
      if (cr->camtrans) {
        double xscale = cr->width / bot_camtrans_get_image_width(cr->camtrans);
        double yscale = cr->width / bot_camtrans_get_image_width(cr->camtrans);
        assert(fabs(xscale - yscale) < 1e-6);
        bot_camtrans_scale_image(cr->camtrans, xscale);
      }
      else {
        printf("%s:%d couldn't find calibration parameters for %s\n", __FILE__, __LINE__, cam_name);
      }
      free(cam_name);
    }
    else {
      printf("%s:%d couldn't find camera parameters for %s\n", __FILE__, __LINE__, channel);
    }

    cr->msg_received = 1;
  }

  if (cr->last_image) {
    bot_core_image_t_destroy(cr->last_image);
  }
  cr->last_image = bot_core_image_t_copy(msg);
  cr->is_uploaded = 0;

  switch (bot_gtk_param_widget_get_enum(cr->pw, PARAM_RENDER_IN)) {
  case RENDER_IN_WIDGET:
    if (gtk_expander_get_expanded(GTK_EXPANDER(cr->expander)))
      bot_gtk_gl_drawing_area_invalidate(cr->gl_area);
  default:
    bot_viewer_request_redraw(self->viewer);
    break;
  }
}
