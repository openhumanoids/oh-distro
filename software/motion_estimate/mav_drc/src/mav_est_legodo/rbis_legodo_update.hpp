#ifndef RBIS_LEGODO_LIB_UPDATE_HPP_
#define RBIS_LEGODO_LIB_UPDATE_HPP_

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>


#include <string>

#include <mav_state_est/mav-est-legodo/rbis_legodo_common.hpp>

#include <estimate/leg_odometry.hpp>
#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <lcmtypes/drc/pose_transform_t.hpp>
#include <lcmtypes/drc/robot_state_t.hpp>

namespace MavStateEst {

class LegOdoHandler {
public:

  LegOdoHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model);

  RBISUpdateInterface * processMessage(const drc::robot_state_t *msg);

  // Calculates the Pelvis Translation between measurements:
  leg_odometry* leg_odo_;
  
  // Converts the Pelvis Translation into a RBIS measurement
  // which is then passed to the estimator
  LegOdoCommon* leg_odo_common_;
};


}
#endif /* RBIS_LEGODO_LIB_UPDATE_HPP_ */

