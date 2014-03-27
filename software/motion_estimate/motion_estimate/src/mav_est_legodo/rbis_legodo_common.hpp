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
  
  BotTrans getTransAsVelocityTrans(BotTrans msgT,
           int64_t utime, int64_t prev_utime);
  
  // Converts the Pelvis Translation into a RBIS measurement
  // which is then passed to the estimator  
  // odometry_status is a measure 0-1 of the reliability of the odometry
  RBISUpdateInterface * createMeasurement(BotTrans &msgT,
           int64_t utime, int64_t prev_utime, float odometry_status);
  
  LegOdoCommonMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_legodo;
  
  Eigen::MatrixXd cov_legodo_uncertain; // equivalent matrix, with higher variance with unreliable odometry
  bool verbose;
};


}
#endif /* RBIS_LEGODO_COMMON_HPP_ */
