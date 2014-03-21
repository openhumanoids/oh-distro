#include "rbis_gpf_update.hpp"
#include <path_util/path_util.h>

namespace MavStateEst {

RBISRgbdGPFMeasurement::RBISRgbdGPFMeasurement(RgbdGPF *gpf_, kinect::frame_msg_t * rgbd_msg_, int64_t utime,
    lcm_t * lcm_, const std::string & pub_channel_) :
    RBISUpdateInterface(rgbd, utime), gpf(gpf_), rgbd_msg(rgbd_msg_), lcm_pub(lcm_), pub_channel(pub_channel_)
{

}
RBISRgbdGPFMeasurement::~RBISRgbdGPFMeasurement()
{
  delete rgbd_msg;
}

void RBISRgbdGPFMeasurement::updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood)
{

//  eigen_dump(prior_state);
//  eigen_dump(prior_cov);

  Eigen::VectorXd z_effective;
  Eigen::MatrixXd R_effective;

  double current_likelihood = 0;

  bool valid = gpf->getMeasurement(prior_state, prior_cov, rgbd_msg, z_effective, R_effective);
  if (valid) {

    //publish the gpf result for logging
    mav_indexed_measurement_t * gpf_msg = gpfCreateLCMmsg(gpf->rgbd_gpf_measurement_indices, z_effective, R_effective);
    gpf_msg->utime = utime;
    gpf_msg->state_utime = prior_state.utime;
    mav_indexed_measurement_t_publish(lcm_pub, pub_channel.c_str(), gpf_msg);
    mav_indexed_measurement_t_destroy(gpf_msg);

// apply the gpf update
    RBIS dstate;
    RBIM dcov;
    current_likelihood = indexedMeasurement(z_effective, R_effective, gpf->rgbd_gpf_measurement_indices, prior_state, prior_cov, dstate,
        dcov);
    rbisApplyDelta(prior_state, prior_cov, dstate, dcov, posterior_state, posterior_covariance);

//    eigen_dump(z_effective.transpose());
//    eigen_dump(R_effective);
//
//    eigen_dump(posterior_state);
//    eigen_dump(posterior_covariance);

  }
  else { //skip this update
    posterior_state = prior_state;
    posterior_covariance = prior_cov;
  }

  loglikelihood = prior_loglikelihood + current_likelihood;
}

RgbdGPFHandler::RgbdGPFHandler(lcm_t * lcm, BotParam * param, BotFrames * frames)
{
  lcm_pub = lcm;
  gpf = new RgbdGPF(lcm, param, frames);

  /// Insert out of process stuff here..

  char * rgbd_chan = bot_param_get_str_or_fail(param, "state_estimator.rgbd_gpf.channel");
  rgbd_channel = rgbd_chan;
  free(rgbd_chan);

}

RBISUpdateInterface * RgbdGPFHandler::processMessage(const kinect::frame_msg_t * msg)
{
//  if (counter++ % downsample_factor != 0)
//    return NULL;

  // mfallon utime - utime_delay changed to timestamp
  kinect::frame_msg_t * msg_cpy = new kinect::frame_msg_t(*msg);
  return new RBISRgbdGPFMeasurement(gpf, msg_cpy, msg->timestamp, lcm_pub, pub_channel);
}

}
