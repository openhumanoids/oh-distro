#ifndef __POSE_MEAS_HANDLER_HPP__
#define __POSE_MEAS_HANDLER_HPP__

//#include "rbis_update_interface.hpp"
#include <mav_state_est/lcm_front_end.hpp>
//#include "lcm_front_end.hpp"
//#include "rbis_initializer.hpp"

#include <eigen_utils/eigen_utils.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>


namespace MavStateEst {

class PoseMeasHandler {
public:
  typedef enum {
    MODE_POSITION, MODE_POSITION_ORIENT
  } PoseMeasMode;

  PoseMeasHandler(BotParam * param, BotFrames *frames);
  PoseMeasHandler(BotParam * param, PoseMeasMode pose_meas_mode);
  void init(BotParam * param);
  RBISUpdateInterface * processMessage(const bot_core::pose_t * msg);
  bool processMessageInit(const bot_core::pose_t * msg, const std::map<std::string, bool> & sensors_initialized
        , const RBIS & default_state, const RBIM & default_cov,
        RBIS & init_state, RBIM & init_cov);

  int no_corrections; // no of corrections to make before going silent
  PoseMeasMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_pose_meas;
};

}//namespace

#endif /* __POSE_MEAS_HANDLER_HPP__ */
