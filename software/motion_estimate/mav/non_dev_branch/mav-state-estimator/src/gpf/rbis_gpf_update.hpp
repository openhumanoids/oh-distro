#ifndef RBIS_GPF_UPDATE_HPP_
#define RBIS_GPF_UPDATE_HPP_

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <mav_state_est/rbis_update_interface.hpp>
#include <mav_state_est/sensor_handlers.hpp>
#include <mav_state_est/gpf/laser_gpf_lib.hpp>

class RBISLaserGPFMeasurement: public RBISUpdateInterface {
public:
  LaserGPF *gpf;
  bot_core_planar_lidar_t * laser_msg;
  lcm_t * lcm;
  std::string pub_channel;

  RBISLaserGPFMeasurement(LaserGPF *gpf_, bot_core_planar_lidar_t * laser_msg_, int64_t utime, lcm_t * lcm_, const std::string & pub_channel_);
  virtual ~RBISLaserGPFMeasurement();

  void updateFilter(const RBIS & prior_state, const RBIM & prior_cov, double prior_loglikelihood);
};

class LaserGPFHandler: public SensorHandler {
public:
  LaserGPFHandler(lcm_t * lcm, BotParam * param, BotFrames * frames);

  RBISUpdateInterface * processMessage(const bot_core_planar_lidar_t * msg);

  lcm_t * lcm;
  std::string laser_channel;
  std::string map_name;
  LaserGPF *gpf;
};

#endif /* RBIS_GPF_UPDATE_HPP_ */
