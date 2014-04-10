#ifndef __rgbd_gpg_h__
#define __rgbd_gpg_h__

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

#include <lcmtypes/kinect.hpp>


#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_utils/eigen_utils.hpp>

#include <mav_state_est/rbis.hpp>
#include <mav_state_est/gpf/gpf.hpp>

#include "RgbdLikelihoodInterface.hpp"

#define RAD2DEG(X) (180.0/M_PI*(X))
#define DEG2RAD(X) (M_PI/180.0*(X))

namespace MavStateEst {

class RgbdGPF: public GPFLikelihoodInterface<RgbdGPF> {
public:

  typedef enum {
    pos_only = 0,
    pos_yaw,
    pos_chi,
    all_states,
    z_only,
    xy_only,
    num_substates,
  } rgbd_gpf_substate;

  static const std::string rgbd_gpf_substate_stings[num_substates];

  bool verbose; //defaults to false
  bool motion_project; //defaults to true

  int beam_skip;
  double spatial_decimation_min;
  double spatial_decimation_max;

  Laser_projector * laser_projector;

  RgbdLikelihoodInterface * laser_like_iface;

  //these need to be public for publishing GPF measurement messages
  rgbd_gpf_substate gpf_substate_mode;
  Eigen::VectorXi rgbd_gpf_measurement_indices;

  RgbdGPF(lcm_t * lcm, BotParam * param, BotFrames * frames);

  virtual ~RgbdGPF();

  double likelihoodFunction(const RBIS & state);

  bool getMeasurement(const RBIS & state, const RBIM & cov, const kinect::frame_msg_t * laser_msg,
      Eigen::VectorXd & z_effective, Eigen::MatrixXd & R_effective);

private:
  void RgbdGPFBaseConstructor(int num_samples, bool gpf_vis, RgbdLikelihoodInterface * likelihood_interface,
      rgbd_gpf_substate gpf_orientation_mode, lcm_t * lcm, BotParam * param, BotFrames * frames);

  Eigen::Matrix3d R_body_to_rgbd; //used to transform body velocities into laser frame for motion projection

  int num_samples;
  double max_weight_proportion;


  laser_projected_scan * projected_laser_scan;
  bot_lcmgl_t * lcmgl_rgbd;
  bot_lcmgl_t * lcmgl_particles_rgbd;

  BotParam * param;
  BotFrames * frames;
  lcm_t * lcm;
};

}

#endif

