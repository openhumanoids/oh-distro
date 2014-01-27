#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <eigen_utils/eigen_utils.hpp>

#include "mav_state_est_renderers.h"
#include <mav_state_est/rbis.hpp>
#include <Eigen/Dense>
#include <laser_utils/laser_util.h>
#include <bot_param/param_util.h>
#include <lcmtypes/mav/indexed_measurement_t.hpp>

#define RENDERER_NAME "Mav State Estimator"

#define PARAM_INITIALIZE "Initialize"

typedef enum {
  GPS_INIT, VICON_INIT, GUI_INIT, ORIGIN_INIT
} init_source_enum;

#define INS_N_SAMPLES_INIT 100

typedef struct _RendererMavStateEst RendererMavStateEst;

using namespace Eigen;
using namespace eigen_utils;
using namespace MavStateEst;

//--------------variables for visualizing filter state--------------------
#define PARAM_SHOW_CHI_COV "Chi cov"
#define PARAM_SHOW_XYZ_COV "xyz cov"
#define PARAM_SHOW_VECS "pose vecs"
#define PARAM_SHOW_VECS_COV "vec cov"
#define PARAM_SHOW_INS "ins vecs"
#define PARAM_SHOW_GPF_MEASUREMENT "gpf z_effective"
#define PARAM_NSIGMA "nsigma"
#define PARAM_AXES_SCALE "axes scale"

#define POSE_ACCEL_COLOR 1, 1, 0, 1
#define POSE_ROTATION_COLOR 0, 1, 1, 1
#define POSE_VEL_COLOR 1, 0, 1, 1

#define COV_XYZ_COLOR 1, 0, 0, 1

#define ACCEL_SCALE 0.1
#define OMEGA_SCALE 3.0
#define MAG_SCALE 1.0
//-------------------------------------------------------------------------

struct _RendererMavStateEst {
  BotRenderer renderer;
  BotEventHandler ehandler;
  BotViewer *viewer;
  lcm_t *lcm;
  BotParam * param;
  BotFrames * frames;

  BotGtkParamWidget *pw;

  //position clicking state
  int click_init_step;
  Vector2d center_click_xy;
  Vector2d heading_release_xy;
  int initializing;
  int64_t init_start_utime;
  int64_t init_utime;

  //current filter state for rendering
  RBIS filter_state;
  RBIM filter_cov;
  double nsigma; //covariance scaling

  //GPF variables (could really be any indexed measurement)
  MatrixXd gpf_R_effective;
  VectorXd gpf_z_effective;
  VectorXi gpf_indices;

  char * pose_render_frame;
};

static void gpf_measurement_message_handler(const lcm_recv_buf_t *rbuf, const char * channel,
    const mav_indexed_measurement_t * msg, void * user)
{
  _RendererMavStateEst *self = (_RendererMavStateEst *) user;

  self->gpf_R_effective.resize(msg->measured_dim, msg->measured_dim);
  self->gpf_z_effective.resize(msg->measured_dim);

  Map<MatrixXd> R_map(msg->R_effective, msg->measured_dim, msg->measured_dim);
  Map<VectorXd> z_map(msg->z_effective, msg->measured_dim);
  Map<VectorXi> index_map(msg->z_indices, msg->measured_dim);
  self->gpf_R_effective = R_map;
  self->gpf_z_effective = z_map;
  self->gpf_indices = index_map;
  bot_viewer_request_redraw(self->viewer);
}

static void filter_state_message_handler(const lcm_recv_buf_t *rbuf, const char * channel,
    const mav_filter_state_t * msg, void * user)
{
  _RendererMavStateEst *self = (_RendererMavStateEst *) user;
  Map<const RBIM> cov_map(msg->cov);
  self->filter_cov = cov_map;
  self->filter_state = RBIS(msg);
  bot_viewer_request_redraw(self->viewer);
}

/**
 * draws acceleration, velocity, angular velocity vectors
 */
static void pose_vecs(_RendererMavStateEst * self)
{
  BotTrans body_trans;
  bot_frames_get_trans(self->frames, self->pose_render_frame , "local", &body_trans); // pose_render_frame was "body"

  // Safety check
  if (hasNan(self->filter_state.acceleration()) ||
      hasNan(self->filter_state.angularVelocity()) ||
      hasNan(self->filter_state.velocity()))
    return;

  glPushMatrix();
  //get to the vehicle CG at correct orientation
  bot_gl_mult_BotTrans(&body_trans);
  glColor4d(POSE_ACCEL_COLOR);
  bot_gl_draw_vector(self->filter_state.acceleration() * ACCEL_SCALE);

  glColor4d(POSE_ROTATION_COLOR);
  bot_gl_draw_vector(self->filter_state.angularVelocity() * OMEGA_SCALE);

  glColor4d(POSE_VEL_COLOR);
  bot_gl_draw_vector(self->filter_state.velocity());

  glPopMatrix();
}

/**
 * draw covariances for the above vectors
 */
static void pose_vecs_cov(_RendererMavStateEst * self)
{
  BotTrans body_trans;
  bot_frames_get_trans(self->frames, self->pose_render_frame, "local", &body_trans); // pose_render_frame was "body"

  Eigen::Matrix3d vb_cov = self->filter_cov.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind);
  Eigen::Matrix3d omega_cov = self->filter_cov.block<3, 3>(RBIS::angular_velocity_ind, RBIS::angular_velocity_ind);
  Eigen::Matrix3d accel_cov = self->filter_cov.block<3, 3>(RBIS::acceleration_ind, RBIS::acceleration_ind);

  // Safety check
  if (hasNan(self->filter_state.acceleration()) ||
      hasNan(self->filter_state.angularVelocity()) ||
      hasNan(self->filter_state.velocity()) ||
      hasNan(vb_cov) ||
      hasNan(omega_cov) ||
      hasNan(accel_cov))
    return;

  glPushMatrix();
  //get to the vehicle CG at correct orientation
  bot_gl_mult_BotTrans(&body_trans);
  glColor4d(POSE_ACCEL_COLOR);
  bot_gl_cov_ellipse_3d(accel_cov, self->filter_state.acceleration() * ACCEL_SCALE, ACCEL_SCALE * self->nsigma);

  glColor4d(POSE_ROTATION_COLOR);
  bot_gl_cov_ellipse_3d(omega_cov, self->filter_state.angularVelocity() * ACCEL_SCALE, ACCEL_SCALE * self->nsigma);

  glColor4d(POSE_VEL_COLOR);
  bot_gl_cov_ellipse_3d(vb_cov, self->filter_state.velocity(), self->nsigma);

  glPopMatrix();
}

static void chi_cov_draw(_RendererMavStateEst * self)
{
  Eigen::Matrix3d chi_cov = self->filter_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind);
  double axes_scale = bot_gtk_param_widget_get_double(self->pw, PARAM_AXES_SCALE);

  // Safety check
  if (hasNan(self->filter_state.position()) || hasNan(chi_cov))
    return;

  glPushMatrix();
  //orient in body frame
  bot_gl_mult_quat_pos(self->filter_state.quat, self->filter_state.position());
  glScaled(axes_scale, axes_scale, axes_scale);
  glLineWidth(3);

  bot_gl_draw_axes();
  bot_gl_draw_axes_cov(chi_cov, self->nsigma);

  glPopMatrix();
}

static void pos_cov_draw(_RendererMavStateEst * self)
{
  Eigen::Matrix3d xyz_cov = self->filter_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind);

  // Safety check
  if (hasNan(self->filter_state.position()) || hasNan(xyz_cov))
    return;

  glColor4d(COV_XYZ_COLOR);
  bot_gl_cov_ellipse_3d(xyz_cov, self->filter_state.position(), self->nsigma);
}

void gpf_measurement_draw(_RendererMavStateEst * self)
{
  Eigen::Matrix3d R_xyz_effective = Eigen::Matrix3d::Zero();
  Eigen::Vector3d z_xyz_effective = self->filter_state.position();

  // Safety check
  if (hasNan(self->filter_state.position()))
    return;

  int xyz_mapping[3] = { -1 };

  for (int ii = 0; ii < 3; ii++) {
    for (int kk = 0; kk < self->gpf_indices.rows(); kk++) {
      if (self->gpf_indices(kk) == RBIS::position_ind + ii) {
        xyz_mapping[ii] = kk;
        break;
      }
    }
  }

  for (int ii = 0; ii < 3; ii++) {
    if (xyz_mapping[ii] < 0)
      continue;

    z_xyz_effective(ii) = self->gpf_z_effective(xyz_mapping[ii]);

    for (int jj = 0; jj < 3; jj++) {
      if (xyz_mapping[jj] < 0)
        continue;
      R_xyz_effective(ii, jj) = self->gpf_R_effective(xyz_mapping[ii], xyz_mapping[jj]);
    }
  }

  glColor3f(0, 0, 1);
  glLineWidth(3);
  bot_gl_cov_ellipse_3d(R_xyz_effective, z_xyz_effective, self->nsigma);
}

static void filter_draw(_RendererMavStateEst *self)
{
  glEnable(GL_DEPTH_TEST);

  if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_VECS)) {
    pose_vecs(self);
  }

  if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_VECS_COV)) {
    pose_vecs_cov(self);
  }

  if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_CHI_COV)) {
    chi_cov_draw(self);
  }

  if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_XYZ_COV)) {
    pos_cov_draw(self);
  }

  if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_GPF_MEASUREMENT)) {
    gpf_measurement_draw(self);
  }

  glDisable(GL_DEPTH_TEST);
}

static void
viewer_meas_draw(RendererMavStateEst *self)
{
  int64_t now = bot_timestamp_now();

  if (self->initializing && self->click_init_step != 0) {

    Vector2d heading = self->heading_release_xy - self->center_click_xy;
    double radius = heading.norm();

    glPushMatrix();
    glLineWidth(3);
    glColor3f(0, 0, 1);
    glTranslated(self->center_click_xy(0), self->center_click_xy(1), 0);
    bot_gl_draw_circle(radius);
    glLineWidth(3);
    glColor3f(0, 1, 0);
    glBegin(GL_LINES);
    glVertex2d(0, 0);
    glVertex2dv(heading.data());
    glEnd();

    glPopMatrix();
  }

}

static void on_draw(BotViewer *viewer, BotRenderer *renderer)
{
  RendererMavStateEst *self = (RendererMavStateEst*) renderer;
  filter_draw(self);
  viewer_meas_draw(self);
}

void computeClickLocation(const double ray_start[3],
    const double ray_dir[3], double click_xy[2])
{
  double dist_to_ground = -ray_start[2] / ray_dir[2];
  click_xy[0] = ray_start[0] + dist_to_ground * ray_dir[0];
  click_xy[1] = ray_start[1] + dist_to_ground * ray_dir[1];
}

static int
mouse_press(BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3],
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererMavStateEst *self = (RendererMavStateEst*) ehandler->user;

  if (!self->initializing)
    return 0;

  if (event->button != 1)
    return 0;

  self->click_init_step = 1;

  Vector2d click_xy;
  computeClickLocation(ray_start, ray_dir, click_xy.data());
  self->center_click_xy = click_xy;

  bot_viewer_request_redraw(self->viewer);
  return 1;
}

static int mouse_motion(BotViewer *viewer, BotEventHandler *ehandler,
    const double ray_start[3], const double ray_dir[3],
    const GdkEventMotion *event)
{
  RendererMavStateEst *self = (RendererMavStateEst*) ehandler->user;

  if (!self->initializing || self->click_init_step < 0)
    return 0;

  if (self->click_init_step > 0)
    self->click_init_step++;
  Vector2d drag_xy;
  computeClickLocation(ray_start, ray_dir, drag_xy.data());
  self->heading_release_xy = drag_xy;

  bot_viewer_request_redraw(self->viewer);
  return 1;
}

static int
mouse_release(BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3],
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererMavStateEst *self = (RendererMavStateEst*) ehandler->user;

  if (!self->initializing || self->click_init_step < 0)
    return 0;

  if (event->button != 1)
    return 0;

  Vector2d release_xy;
  computeClickLocation(ray_start, ray_dir, release_xy.data());
  self->heading_release_xy = release_xy;

  Vector2d heading_vec = self->heading_release_xy - self->center_click_xy;
  if (self->click_init_step < 10 || heading_vec.norm() < .02) {
    self->click_init_step = 0;
    bot_viewer_set_status_bar_message(self->viewer,
        "Did not get heading. Click and drag to initialize filter state");
    return 0;
  }
  self->click_init_step = -1;

  Vector3d click_xyz;
  click_xyz << self->center_click_xy, 0; //TODO: 0?
  double heading = eigen_utils::atan2Vec(heading_vec);
  double click_xy_sigma = heading_vec.norm();

  Eigen::Vector3d click_xyz_cov_diagonal =
      Eigen::Array3d(click_xy_sigma, click_xy_sigma, click_xy_sigma).square();

  mav::indexed_measurement_t meas_msg;
  meas_msg.state_utime = meas_msg.utime = bot_timestamp_now();
  meas_msg.measured_dim = 4;
  meas_msg.measured_cov_dim = bot_sq(meas_msg.measured_dim);
  meas_msg.z_indices.resize(meas_msg.measured_dim);
  meas_msg.z_effective.resize(meas_msg.measured_dim);
  meas_msg.R_effective.resize(meas_msg.measured_cov_dim);

  Eigen::Map<VectorXi> inds_map(&meas_msg.z_indices[0], meas_msg.measured_dim);
  inds_map.head(3) = RBIS::positionInds();
  inds_map(3) = RBIS::chi_ind + 2;

  Eigen::Map<MatrixXd> cov_map(&meas_msg.R_effective[0], meas_msg.measured_dim, meas_msg.measured_dim);
  cov_map.setZero();
  cov_map.topLeftCorner(3, 3).diagonal() = click_xyz_cov_diagonal;
  cov_map(3, 3) = bot_sq(bot_to_radians(5)); //FIXME hard coded meading cov

  Eigen::Map<VectorXd> meas_map(&meas_msg.z_effective[0], 4);
  meas_map.head(3) = click_xyz;
  meas_map(3) = heading;

  lcm::LCM lcm_cpp(self->lcm);
  lcm_cpp.publish("MAV_STATE_EST_VIEWER_MEASUREMENT", &meas_msg);

  bot_viewer_set_status_bar_message(self->viewer, "Init XYZ: %f,%f,%f, yaw: %f", meas_map(0), meas_map(1), meas_map(2),
      bot_to_degrees(meas_map(3)));

  self->initializing = 0;

  bot_viewer_request_redraw(self->viewer);
  return 1;
}

static void
activate(RendererMavStateEst *self)
{

  bot_viewer_set_status_bar_message(self->viewer,
      "Click and drag to initialize filter state");

  self->initializing = 1;
  self->click_init_step = 0;
  self->init_start_utime = bot_timestamp_now();

}

static int key_press(BotViewer *viewer, BotEventHandler *ehandler,
    const GdkEventKey *event)
{
  RendererMavStateEst *self = (RendererMavStateEst*) ehandler->user;

  if ((event->keyval == 'I') && !self->initializing) {
    activate(self);
  }
  else if (event->keyval == GDK_Escape) {
    self->initializing = 0;
    bot_viewer_set_status_bar_message(self->viewer, "");
  }

  return 0;
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererMavStateEst *self = (RendererMavStateEst*) user;
  if (!strcmp(name, PARAM_INITIALIZE)) {
    activate(self);
  }

  self->nsigma = bot_gtk_param_widget_get_double(self->pw, PARAM_NSIGMA);
  bot_viewer_request_redraw(self->viewer);

}

static void
_free(BotRenderer *renderer)
{
  //    RendererMavStateEst *self = (RendererMavStateEst*) renderer;
  free(renderer);
}

static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererMavStateEst *self = (RendererMavStateEst *) user_data;
  bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  RendererMavStateEst *self = (RendererMavStateEst *) user_data;
  bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

BotRenderer *renderer_mav_state_est_new(BotViewer *viewer, int render_priority, lcm_t * lcm,
    BotParam * param, BotFrames * frames)
{
  RendererMavStateEst *self = (RendererMavStateEst*) calloc(1, sizeof(RendererMavStateEst));
  self->viewer = viewer;
  self->renderer.draw = on_draw;
  self->renderer.destroy = _free;
  self->renderer.name = strdup(RENDERER_NAME);
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

  self->lcm = lcm;
  self->param = param;
  self->frames = frames;

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  bot_gtk_param_widget_add_buttons(self->pw, PARAM_INITIALIZE, NULL);

  bot_gtk_param_widget_add_separator(self->pw, "Cov Visualization");

//  gtk_box_pack_start(GTK_BOX(self->pw), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_NSIGMA, BOT_GTK_PARAM_WIDGET_SLIDER, 0, 5, .01, 1);
  bot_gtk_param_widget_add_double(self->pw, PARAM_AXES_SCALE, BOT_GTK_PARAM_WIDGET_SLIDER, 1, 5, .01, 3);

  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_CHI_COV, 1,
      PARAM_SHOW_XYZ_COV,
      1, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_VECS_COV, 1,
      PARAM_SHOW_VECS,
      1, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_GPF_MEASUREMENT, 1, NULL);

  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  self->renderer.widget = GTK_WIDGET(self->pw);

  g_signal_connect(G_OBJECT(viewer), "load-preferences", G_CALLBACK(on_load_preferences), self);
  g_signal_connect(G_OBJECT(viewer), "save-preferences", G_CALLBACK(on_save_preferences), self);

  self->initializing = 0;

  self->gpf_R_effective = MatrixXd::Zero(3, 3);
  self->gpf_z_effective = VectorXd::Zero(3);

  self->filter_state = RBIS();
  self->filter_cov = RBIM();

  // Coordinate Frame at which to renderer the state variables: [add mfallon]
  //self->pose_render_frame = "body";
  self->pose_render_frame = bot_param_get_str_or_fail(param, "state_estimator.pose_render_frame");

  //subscribe to filter, gpf
  mav_filter_state_t_subscribe(lcm, bot_param_get_str_or_fail(param, "state_estimator.filter_state_channel"),
      filter_state_message_handler, self);
  mav_indexed_measurement_t_subscribe(lcm, "GPF_MEASUREMENT", gpf_measurement_message_handler, self);

  return &self->renderer;
}

extern "C" void add_mav_state_est_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t * lcm,
    BotParam * param, BotFrames * frames)
{
  BotRenderer * self = renderer_mav_state_est_new(viewer, render_priority, lcm, param, frames);
  bot_viewer_add_renderer(viewer, self, render_priority);
}
