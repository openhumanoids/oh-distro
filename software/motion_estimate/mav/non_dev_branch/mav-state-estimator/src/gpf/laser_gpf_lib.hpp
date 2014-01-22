#ifndef __laser_gpg_h__
#define __laser_gpg_h__

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <deque>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <octomap_utils/octomap_util.hpp>

#include <lcmtypes/mav_pose_t.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_utils/eigen_utils.hpp>

#include <mav_state_est/rbis.hpp>
#include <mav_state_est/gpf/gpf.hpp>

#include "LaserLikelihoodInterface.hpp"

#define RAD2DEG(X) (180.0/M_PI*(X))
#define DEG2RAD(X) (M_PI/180.0*(X))

class LaserGPF: public GPFLikelihoodInterface<LaserGPF> {
public:

  typedef enum {
    pos_only = 0,
    pos_yaw,
    pos_chi,
    all_states,
    z_only,
    xy_only,
    num_substates,
  } laser_gpf_substate;

  static const std::string laser_gpf_substate_stings[num_substates];

  bool verbose; //defaults to false
  bool motion_project; //defaults to true

  int beam_skip;
  double spatial_decimation_min;
  double spatial_decimation_max;

  Laser_projector * laser_projector;

  LaserLikelihoodInterface * laser_like_iface;

  //these need to be public for publishing GPF measurement messages
  laser_gpf_substate gpf_substate_mode;
  Eigen::VectorXi laser_gpf_measurement_indices;

  LaserGPF(lcm_t * lcm, BotParam * param, BotFrames * frames);

  virtual ~LaserGPF();

  double likelihoodFunction(const RBIS & state);

  bool getMeasurement(const RBIS & state, const RBIM & cov, const bot_core_planar_lidar_t * laser_msg,
      Eigen::VectorXd & z_effective, Eigen::MatrixXd & R_effective);

private:
  void LaserGPFBaseConstructor(int num_samples, bool gpf_vis, LaserLikelihoodInterface * likelihood_interface,
      laser_gpf_substate gpf_orientation_mode, lcm_t * lcm, BotParam * param, BotFrames * frames);

  Eigen::Matrix3d R_body_to_laser; //used to transform body velocities into laser frame for motion projection

  int num_samples;

  laser_projected_scan * projected_laser_scan;
  bot_lcmgl_t * lcmgl_laser;
  bot_lcmgl_t * lcmgl_particles;

  BotParam * param;
  BotFrames * frames;
  lcm_t * lcm;
};

#endif

