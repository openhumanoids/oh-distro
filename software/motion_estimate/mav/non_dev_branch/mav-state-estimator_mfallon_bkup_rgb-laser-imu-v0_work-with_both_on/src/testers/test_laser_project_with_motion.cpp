#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <path_util/path_util.h>
#include <eigen_utils/eigen_utils.hpp>
#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <laser_utils/laser_util.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <getopt.h>

using namespace std;
using namespace Eigen;
using namespace eigen_utils;

#define RAD2DEG(X) (180.0/M_PI*(X))
#define DEG2RAD(X) (M_PI/180.0*(X))

int main(int argc, char **argv)
{
  if (argc != 8) {
    fprintf(stderr, "%s usage: omega_x omega_y omega_z v_x v_y v_z wall_scale\n", argv[0]);
    exit(1);
  }

  double omegab[3] = { 0, 0, 0 };
  double vb[3] = { 0, 0, 0 };

  int counter = 1;
  for (int ii = 0; ii < 3; ii++) {
    omegab[ii] = atof(argv[counter]);
    counter++;
  }

  for (int ii = 0; ii < 3; ii++) {
    vb[ii] = atof(argv[counter]);
    counter++;
  }

  double wall_scale = atof(argv[counter]);

  Map<Vector3d> omega(omegab);
  Map<Vector3d> v(vb);

  eigen_dump(omega);
  eigen_dump(v);

  lcm_t * lcm = lcm_create(NULL);
  BotParam * param = bot_param_new_from_server(lcm, 0);
  if (param == NULL) {
    "start the param server\n";
    exit(1);
  }
  BotFrames * frames = bot_frames_new(lcm, param);

  double neg_y_wall = wall_scale * 1;
  double pos_y_wall = wall_scale * 2;

  Laser_projector * laser_projector = laser_projector_new(param, frames, "dummy_laser", 1);
  char param_prefix[1024]; /* param file path */
  bot_param_get_planar_lidar_prefix(NULL, laser_projector->laser_name, param_prefix, sizeof(param_prefix));

  char key[1024];
  sprintf(key, "%s.angle_range", param_prefix);
  double angle_range[2];
  bot_param_get_double_array_or_fail(param, key, angle_range, 2);

  bot_core_planar_lidar_t laser_msg;
  laser_msg.nranges = laser_projector->surroundRegion[1];
  laser_msg.rad0 = DEG2RAD(angle_range[1]);
  laser_msg.radstep = DEG2RAD(angle_range[0] - angle_range[1]) / ((double) laser_msg.nranges);
  laser_msg.ranges = (float *) calloc(laser_msg.nranges, sizeof(float));
  laser_msg.nintensities = 0;

  for (int ii = 0; ii < laser_msg.nranges; ii++) {
    double angle = laser_msg.rad0 + laser_msg.radstep * ii;

    if (angle < 0)
      laser_msg.ranges[ii] = -neg_y_wall / sin(angle);
    else
      laser_msg.ranges[ii] = pos_y_wall / sin(angle);
  }

//  laser_projector->surroundRegion[0] = 0;
//  laser_projector->surroundRegion[1] = 1000;

  laser_projected_scan * projected_laser_scan = laser_create_projected_scan_from_planar_lidar_with_motion(
      laser_projector, &laser_msg, "enu_global", omegab, vb);

  bot_lcmgl_t * lcmgl = bot_lcmgl_init(lcm, "Laser Motion Project");

  lcmglColor3f(1, 0, 0);
  lcmglPointSize(3);
  lcmglBegin(GL_POINTS);
  for (int ii = 0; ii < laser_msg.nranges; ii++) {
    lcmglVertex3fv(point3d_as_array(&projected_laser_scan->points[ii]));
  }
  lcmglEnd();
  bot_lcmgl_switch_buffer(lcmgl);

  return 0;
}

