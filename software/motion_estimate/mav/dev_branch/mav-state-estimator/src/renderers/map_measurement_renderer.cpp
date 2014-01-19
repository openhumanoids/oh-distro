#include "mav_state_est_renderers.h"
#include <mav_state_est/map_measurement_function.hpp>
#include <mav_state_est/gpf/gpf.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace eigen_utils;
using namespace std;
using namespace octomap;
using namespace MavStateEst;

#define PARAM_PSI "yaw angle"
#define PARAM_PHI "roll angle"
#define PARAM_NSIGMA "nsigma"
#define PARAM_SPATIAL_DECIMATION "xy decimation"
#define PARAM_ALPHA "transparency"
#define PARAM_RED_INFO "red info gain"
#define PARAM_DRAW_HEIGHT "draw_height"
#define PARAM_RENDER_MODE "render mode"
typedef enum {
  COV_RENDER, INFO_RENDER, XY_INFO_RENDER, Z_INFO_RENDER
} render_mode_enum;

#define RENDERER_NAME "Map Measurement Function"

typedef struct _RendererMapMeas RendererMapMeas;

typedef Matrix<double, 9, 9> Matrix9d;

struct _RendererMapMeas {
  BotRenderer renderer;
  BotGtkParamWidget *pw;
  BotViewer *viewer;
  lcm_t * lcm;
  BotParam * param;
  BotFrames * frames;

  render_mode_enum render_mode;
  double z_draw_height;
  double red_info;

  MapMeasurementFunction * map_meas;
  OcTree * ocTree;

};

static void map_measurement_function_handler(const lcm_recv_buf_t *rbuf, const char * channel,
    const mav_map_measurement_function_t * msg, void * user)
{
  RendererMapMeas *self = (RendererMapMeas *) user;

  printf("got new measurement map\n");
  if (self->map_meas != NULL)
    delete self->map_meas;

  self->map_meas = new MapMeasurementFunction();
  self->map_meas->set_from_map_measurement_function_t(msg);

  bot_gtk_param_widget_set_double(self->pw, PARAM_DRAW_HEIGHT, self->map_meas->z_height - .5);
}

static void on_octomap(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_raw_t *msg, void *user_data)
{
  fprintf(stderr, "map meas renderer got new octomap\n");
  RendererMapMeas *self = (RendererMapMeas *) user_data;
  if (self->ocTree != NULL
  )
    delete self->ocTree;
  std::stringstream datastream;
  datastream.write((const char*) msg->data, msg->length);
  self->ocTree = new octomap::OcTree(1); //resolution will be set by data from message
  self->ocTree->readBinary(datastream);
//  double minX, minY, minZ, maxX, maxY, maxZ;
//  self->ocTree->getMetricMin(minX, minY, minZ);
//  self->ocTree->getMetricMax(maxX, maxY, maxZ);
//  printf("\nmap bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]\n", minX, minY, minZ, maxX, maxY, maxZ);

  fprintf(stderr, "loadedOctomap\n");
}

static void map_meas_draw(BotViewer *viewer, BotRenderer *renderer)
{
  RendererMapMeas *self = (RendererMapMeas*) renderer;

  if (self->map_meas == NULL
  ) {
    return;
  }

  self->render_mode = (render_mode_enum) bot_gtk_param_widget_get_enum(self->pw, PARAM_RENDER_MODE);
  double alpha = bot_gtk_param_widget_get_double(self->pw, PARAM_ALPHA);
  self->z_draw_height = bot_gtk_param_widget_get_double(self->pw, PARAM_DRAW_HEIGHT);
  double nsigma = bot_gtk_param_widget_get_double(self->pw, PARAM_NSIGMA);
  self->red_info = bot_gtk_param_widget_get_double(self->pw, PARAM_RED_INFO);
  int spatial_decimation = bot_gtk_param_widget_get_int(self->pw, PARAM_SPATIAL_DECIMATION);

  double phipsi_loc[2], xy_loc[2];
  phipsi_loc[0] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_PHI));
  phipsi_loc[1] = bot_to_radians(bot_gtk_param_widget_get_double(self->pw, PARAM_PSI));

  MapMeasurementFunction::CovPixelMap * xy_cov_map = self->map_meas->phi_psi_xy_cov_map->readValue(phipsi_loc);

  int loc_ind = self->map_meas->phi_psi_xy_cov_map->getInd(phipsi_loc);
  double phi_psi_mapped_loc[2];
  self->map_meas->phi_psi_xy_cov_map->indToLoc(loc_ind, phi_psi_mapped_loc);

  MatrixXf scalar_vals;
  int x_dim = xy_cov_map->dimensions[0];
  int y_dim = xy_cov_map->dimensions[1];

  if (self->render_mode != COV_RENDER) {
    scalar_vals.setConstant(x_dim, y_dim, -1);
  }

//  fprintf(stderr, "phipsi_mapped_loc (%f,%f)\n", bot_to_degrees(phi_psi_mapped_loc[0]),
//      bot_to_degrees(phi_psi_mapped_loc[1]));

  glEnable(GL_DEPTH_TEST);
  glColor3d(1, 0, 0);
  int xy_spatial_inds[2];

  for (int k = 0; k < x_dim; k += spatial_decimation) {
    for (int l = 0; l < y_dim; l += spatial_decimation) {
      xy_spatial_inds[0] = k;
      xy_spatial_inds[1] = l;
      int xy_ind = xy_cov_map->getInd(xy_spatial_inds);
      xy_cov_map->indToLoc(xy_ind, xy_loc);

      //only draw cov in free space
      if (self->ocTree != NULL) {
        octomap::OcTreeNode* node =
            self->ocTree->search(xy_loc[0], xy_loc[1], self->map_meas->z_height);
        if (node == NULL) { //node must be marked as unoccupied
          continue;
        }
        else if (self->ocTree->isNodeOccupied(node)) {
          continue;
        }
      }

      Matrix3d cov = xy_cov_map->readValue(xy_spatial_inds);

      switch (self->render_mode) {
      case COV_RENDER:
        {
//          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov);
//          Eigen::Vector3d eig_vals = eigen_solver.eigenvalues();
//
//          if ((eig_vals.array() > .5 * GPF_R_NEG_EIG_CORRECTION).all()) //don't render covs with no information
//            continue;

          Vector3d pos;
          pos << xy_loc[0], xy_loc[1], self->z_draw_height;
          bot_gl_cov_ellipse_3d(cov, pos, nsigma);
          break;
        }

      case INFO_RENDER:
        {
          scalar_vals(k, l) = cov.inverse().trace();
          break;
        }
      case XY_INFO_RENDER:
        {
          scalar_vals(k, l) = cov.topLeftCorner(2, 2).inverse().trace();
          break;
        }
      case Z_INFO_RENDER:
        {
          scalar_vals(k, l) = cov.bottomRightCorner(1, 1).inverse().trace();
          break;
        }
      }
    }
  }

  if (self->render_mode != COV_RENDER) {

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //        glEnable(GL_LIGHTING);

//    double max_scalar = scalar_vals.array().maxCoeff();
//
//    double max_scalar = nsigma;
//    scalar_vals = scalar_vals / max_scalar;
    double res = spatial_decimation * xy_cov_map->metersPerPixel / 2;

    for (int k = 0; k < x_dim; k += spatial_decimation) {
      for (int l = 0; l < y_dim; l += spatial_decimation) {
        if (scalar_vals(k, l) < 0)
          continue;

        xy_spatial_inds[0] = k;
        xy_spatial_inds[1] = l;
        int xy_ind = xy_cov_map->getInd(xy_spatial_inds);
        xy_cov_map->indToLoc(xy_ind, xy_loc);

        float * color = bot_color_util_jet(scalar_vals(k, l) / self->red_info);
        glColor4f(color[0], color[1], color[2], alpha);
        glBegin(GL_POLYGON);
        glVertex3d(xy_loc[0] + res, xy_loc[1] + res, self->z_draw_height);
        glVertex3d(xy_loc[0] + res, xy_loc[1] - res, self->z_draw_height);
        glVertex3d(xy_loc[0] - res, xy_loc[1] - res, self->z_draw_height);
        glVertex3d(xy_loc[0] - res, xy_loc[1] + res, self->z_draw_height);
        glEnd();
      }
    }
    glPopAttrib();
  }
  glDisable(GL_DEPTH_TEST);
}

static void map_meas_free(BotRenderer *renderer)
{
  RendererMapMeas *self = (RendererMapMeas*) renderer;
  free(self);
}

static void on_clear_button(GtkWidget *button, RendererMapMeas *self)
{
  if (!self->viewer)
    return;

  if (self->map_meas != NULL
  )
    delete self->map_meas;
  self->map_meas = NULL;

  bot_viewer_request_redraw(self->viewer);
}

static void on_clear_octo(GtkWidget *button, RendererMapMeas *self)
{
  if (!self->viewer)
    return;

  if (self->ocTree != NULL
  )
    delete self->ocTree;
  self->ocTree = NULL;

  bot_viewer_request_redraw(self->viewer);
}

static void on_load_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  if (!viewer)
    return;
  RendererMapMeas *self = (RendererMapMeas *) user_data;
  bot_gtk_param_widget_load_from_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_save_preferences(BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
  if (!viewer)
    return;
  RendererMapMeas *self = (RendererMapMeas *) user_data;
  bot_gtk_param_widget_save_to_key_file(self->pw, keyfile, RENDERER_NAME);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererMapMeas *self = (RendererMapMeas*) user;
  BotViewer *viewer = self->viewer;

  bot_viewer_request_redraw(self->viewer);
}

BotRenderer *renderer_MapMeas_new(BotViewer *viewer, int render_priority,
    lcm_t * lcm, BotParam * param, BotFrames * frames)
{
  RendererMapMeas *self = (RendererMapMeas*) calloc(1, sizeof(RendererMapMeas));
  BotRenderer *renderer = &self->renderer;
  self->param = param;
  self->frames = frames;
  self->lcm = lcm;
  self->viewer = viewer;

  renderer->draw = map_meas_draw;
  renderer->destroy = map_meas_free;
  renderer->widget = gtk_vbox_new(FALSE, 0);
  renderer->name = (char *) RENDERER_NAME;
  renderer->user = self;
  renderer->enabled = 1;

  renderer->widget = gtk_vbox_new(false, 0);
  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_PSI, BOT_GTK_PARAM_WIDGET_SPINBOX, -180, 180, 1, 0);

  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_PHI, BOT_GTK_PARAM_WIDGET_SPINBOX, -90, 90, 1, 0);

  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_NSIGMA, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 3, .01, 1);

  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_RED_INFO, BOT_GTK_PARAM_WIDGET_SPINBOX, .01, 10, .01, 1);

  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_ALPHA, BOT_GTK_PARAM_WIDGET_SPINBOX, 0, 1, .01, .5);

  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_double(self->pw, PARAM_DRAW_HEIGHT, BOT_GTK_PARAM_WIDGET_SPINBOX, -5, 20, .5, 3);

  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_int(self->pw, PARAM_SPATIAL_DECIMATION, BOT_GTK_PARAM_WIDGET_SPINBOX, 1, 10, 1, 1);

  bot_gtk_param_widget_add_enum(self->pw, PARAM_RENDER_MODE, BOT_GTK_PARAM_WIDGET_DEFAULTS, COV_RENDER,
      "covariances", COV_RENDER,
      "info gain", INFO_RENDER,
      "xy info", XY_INFO_RENDER,
      "z info", Z_INFO_RENDER,
      NULL);

  GtkWidget *clear_button = gtk_button_new_with_label("Clear Meas");
  gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 0);
  g_signal_connect(G_OBJECT(clear_button), "clicked", G_CALLBACK(on_clear_button), self);

  GtkWidget *clear_octo = gtk_button_new_with_label("Clear Octomap");
  gtk_box_pack_start(GTK_BOX(renderer->widget), clear_octo, FALSE, FALSE, 0);
  g_signal_connect(G_OBJECT(clear_octo), "clicked", G_CALLBACK(on_clear_octo), self);

  gtk_widget_show_all(renderer->widget);

  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  on_param_widget_changed(self->pw, "", self);

  //  g_signal_connect (G_OBJECT (self->pw), "changed",
  //      G_CALLBACK (on_param_widget_changed), self);

  g_signal_connect(G_OBJECT (viewer), "load-preferences",
      G_CALLBACK (on_load_preferences), self);
  g_signal_connect(G_OBJECT (viewer), "save-preferences",
      G_CALLBACK (on_save_preferences), self);

  self->map_meas = NULL;
  self->ocTree = NULL;

  //FIXME hack, since it's too big for lc
//  self->map_meas = new MapMeasurementFunction();
//  self->map_meas->loadFromFile("/home/abry/Fixie/build/data/lr3tt_rrbt/octomap.bt_meas_map5"); //hack since it's too big for lcm
//  bot_gtk_param_widget_set_double(self->pw, PARAM_DRAW_HEIGHT, self->map_meas->z_height);

  mav_map_measurement_function_t_subscribe(lcm, "FIXIE_MAP_MEAS", map_measurement_function_handler, self);
  bot_core_raw_t_subscribe(lcm, "OCTOMAP", on_octomap, self);
  return &self->renderer;
}

extern "C" void add_map_measurement_renderer_to_viewer(BotViewer *viewer, int render_priority,
    lcm_t * lcm, BotParam * param, BotFrames * frames)
{
  BotRenderer * self = renderer_MapMeas_new(viewer, render_priority, lcm, param, frames);
  bot_viewer_add_renderer(viewer, self, render_priority);
}
