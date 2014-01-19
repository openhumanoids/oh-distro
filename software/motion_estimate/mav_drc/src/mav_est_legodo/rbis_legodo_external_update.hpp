#ifndef RBIS_LEGODO_EXTERNAL_LIB_UPDATE_HPP_
#define RBIS_LEGODO_EXTERNAL_LIB_UPDATE_HPP_

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <mav_state_est/mav-est-legodo/rbis_legodo_common.hpp>

#include <lcmtypes/drc/pose_transform_t.hpp>

namespace MavStateEst {

class LegOdoExternalHandler {
public:
  
  LegOdoExternalHandler(BotParam * param);

  RBISUpdateInterface * processMessage(const drc::pose_transform_t  * msg);

  // Converts the Pelvis Translation into a RBIS measurement
  // which is then passed to the estimator
  LegOdoCommon* leg_odo_common_;
};


}
#endif /* RBIS_LEGODO_EXTERNAL_LIB_UPDATE_HPP_ */

