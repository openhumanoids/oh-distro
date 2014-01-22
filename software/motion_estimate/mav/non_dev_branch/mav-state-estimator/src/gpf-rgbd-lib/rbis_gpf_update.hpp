#ifndef RBIS_GPF_RGBD_LIB_UPDATE_HPP_
#define RBIS_GPF_RGBD_LIB_UPDATE_HPP_

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <lcmtypes/kinect_frame_msg_t.h>

#include <mav_state_est/rbis_update_interface.hpp>
#include <mav_state_est/sensor_handlers.hpp>
#include <mav_state_est/gpf-rgbd-lib/rgbd_gpf_lib.hpp>

class RBISRgbdGPFMeasurement: public RBISUpdateInterface {
public:
  RgbdGPF *gpf;
  kinect_frame_msg_t * laser_msg;
  lcm_t * lcm;
  std::string pub_channel;

  RBISRgbdGPFMeasurement(RgbdGPF *gpf_, kinect_frame_msg_t * laser_msg_, int64_t utime, lcm_t * lcm_, const std::string & pub_channel_);
  virtual ~RBISRgbdGPFMeasurement();

  void updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood);
};

class RgbdGPFHandler: public SensorHandler {
public:
  RgbdGPFHandler(lcm_t * lcm, BotParam * param, BotFrames * frames);

  RBISUpdateInterface * processMessage(const kinect_frame_msg_t * msg);

  lcm_t * lcm;
  std::string rgbd_channel;
  std::string map_name;
  RgbdGPF *gpf;
};

#endif /* RBIS_GPF_UPDATE_HPP_ */
