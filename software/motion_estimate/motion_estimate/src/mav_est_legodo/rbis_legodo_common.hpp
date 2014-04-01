#ifndef RBIS_LEGODO_COMMON_HPP_
#define RBIS_LEGODO_COMMON_HPP_

#include <mav_state_est/rbis_update_interface.hpp>
#include <mav_state_est/sensor_handlers.hpp>

namespace MavStateEst {

class LegOdoCommon {
public:
  // Typical mode is MODE_LIN_RATE
  typedef enum {
    MODE_LIN_RATE, MODE_ROT_RATE, MODE_LIN_AND_ROT_RATE, MODE_POSITION_AND_LIN_RATE
  } LegOdoCommonMode;  
  
  LegOdoCommon(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, BotParam * param);
  lcm::LCM* lcm_pub;
  lcm::LCM* lcm_recv;
  
  BotTrans getTransAsVelocityTrans(BotTrans msgT,
           int64_t utime, int64_t prev_utime);
  
  
  // Determine the relevent covariance:
  void getCovariance(LegOdoCommonMode mode_current, bool delta_certain,
           Eigen::MatrixXd &cov_legodo, Eigen::VectorXi &z_indices);
  
  // Converts the Pelvis Position and Delta Translation into a combined RBIS measurement
  // which is then passed to the estimator  
  // odo_position_status is a position boolean validity flag
  // odo_delta_status is a measure 0-1 of the reliability of the delta odometry
  RBISUpdateInterface * createMeasurement(BotTrans &odo_positionT, BotTrans &delta_odoT,
           int64_t utime, int64_t prev_utime, 
           int odo_position_status, float odo_delta_status);
  
  LegOdoCommonMode mode_;
  //Eigen::VectorXi z_indices;
  // Eigen::MatrixXd cov_legodo;
  //Eigen::MatrixXd cov_legodo_uncertain; // equivalent matrix, with higher variance with unreliable odometry
  bool verbose;
  
  double R_legodo_xyz_;
  double R_legodo_vxyz_;
  double R_legodo_vang_;
  double R_legodo_vxyz_uncertain_;
  double R_legodo_vang_uncertain_;
  
};


}
#endif /* RBIS_LEGODO_COMMON_HPP_ */
