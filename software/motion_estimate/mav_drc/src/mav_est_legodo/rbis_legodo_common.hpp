#ifndef RBIS_LEGODO_COMMON_HPP_
#define RBIS_LEGODO_COMMON_HPP_

#include <mav_state_est/rbis_update_interface.hpp>
#include <mav_state_est/sensor_handlers.hpp>

namespace MavStateEst {

class LegOdoCommon {
public:
  // Typical mode is MODE_LIN_AND_ROT_RATE
  typedef enum {
    MODE_LIN_RATE, MODE_ROT_RATE, MODE_LIN_AND_ROT_RATE
  } LegOdoCommonMode;  
  
  LegOdoCommon(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, BotParam * param);
  lcm::LCM* lcm_pub;
  lcm::LCM* lcm_recv;
  
  
  // Converts the Pelvis Translation into a RBIS measurement
  // which is then passed to the estimator  
  RBISUpdateInterface * createMeasurement(BotTrans &msgT,
           int64_t utime, int64_t prev_utime);
  
  LegOdoCommonMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_legodo;
  bool verbose;
};


}
#endif /* RBIS_LEGODO_COMMON_HPP_ */
