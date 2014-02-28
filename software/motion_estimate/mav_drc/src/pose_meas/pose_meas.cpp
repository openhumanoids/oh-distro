#include "pose_meas.hpp"

using namespace Eigen;

namespace MavStateEst {

PoseMeasHandler::PoseMeasHandler(BotParam * param, BotFrames * frames)
{
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.pose_meas.mode");

  if (strcmp(mode_str, "position") == 0) {
    mode = PoseMeasHandler::MODE_POSITION;
    std::cout << "PoseMeas will provide position measurements." << std::endl;
  }
  else if (strcmp(mode_str, "position_orient") == 0) {
    mode = PoseMeasHandler::MODE_POSITION_ORIENT;
    std::cout << "PoseMeas will provide position and orientation measurements." << std::endl;
  }
  else {
    mode = PoseMeasHandler::MODE_POSITION;
    std::cout << "Unrecognized PoseMeas mode. Using position mode by default." << std::endl;
  }

  no_corrections = bot_param_get_int_or_fail(param, "state_estimator.pose_meas.no_corrections");
  std::cout << "Apply " << no_corrections << " PoseMeas corrections before going silent." << std::endl;

  free(mode_str);
  init(param);
}

PoseMeasHandler::PoseMeasHandler(BotParam * param, PoseMeasMode pose_meas_mode)
{
  init(param);
}

void PoseMeasHandler::init(BotParam * param)
{
  // Build full covariance matrix - we may only use part of it.
  double r_pose_meas_xyz = bot_param_get_double_or_fail(param, "state_estimator.pose_meas.r_xyz");
  double r_pose_meas_chi = bot_param_get_double_or_fail(param, "state_estimator.pose_meas.r_chi");

  cov_pose_meas = Eigen::MatrixXd::Zero(6, 6);
  cov_pose_meas.topLeftCorner<3, 3>() = pow(r_pose_meas_xyz, 2) * Eigen::Matrix3d::Identity();
  cov_pose_meas.bottomRightCorner<3, 3>() = pow(bot_to_radians(r_pose_meas_chi), 2) * Eigen::Matrix3d::Identity();

  if (mode == MODE_POSITION) {
    z_indices = RBIS::positionInds();
  }
  else {
    z_indices.resize(6);
    z_indices.head<3>() = RBIS::positionInds();
    z_indices.tail<3>() = RBIS::chiInds();
  }
}

RBISUpdateInterface * PoseMeasHandler::processMessage(const bot_core::pose_t * msg)
{
  // If we have created no_corrections, go silent afterwards
  no_corrections--;
  if (no_corrections==1)
    std::cout << "Finished making PoseMeas corrections\n";
  if (no_corrections <= 0){
    //std::cout << "Finished making PoseMeas corrections <<<<<<<<<<<<<<<<<<<<<\n";
    return NULL;
  }  
  //std::cout << "About to make a PoseMeas correction <<<<<<<<<<<<<<<<<<<<<\n";

  BotTrans local_to_body;
  memset(&local_to_body, 0, sizeof(local_to_body));
  memcpy(local_to_body.trans_vec, msg->pos, 3*sizeof(double));
  memcpy(local_to_body.rot_quat, msg->orientation, 4*sizeof(double));
  
  int64_t utime = msg->utime;
  
  if ((Eigen::Map<const Eigen::Array3d>( local_to_body.trans_vec ).abs() < 1e-5).all())
    return NULL;

  if (mode == MODE_POSITION) {
    return new RBISIndexedMeasurement(z_indices, Eigen::Map<const Eigen::Vector3d>( local_to_body.trans_vec ),
        cov_pose_meas.block<3, 3>(0, 0), RBISUpdateInterface::pose_meas,
        utime);
  }
  else {
    Eigen::VectorXd z_meas(6);
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, local_to_body.rot_quat );

    z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(local_to_body.trans_vec);

    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_pose_meas, quat, RBISUpdateInterface::pose_meas,
        utime);
  }
}

bool PoseMeasHandler::processMessageInit(const bot_core::pose_t * msg,
    const std::map<std::string, bool> & sensors_initialized
    , const RBIS & default_state, const RBIM & default_cov,
    RBIS & init_state, RBIM & init_cov)
{

  BotTrans local_to_body;
  memset(&local_to_body, 0, sizeof(local_to_body));
  memcpy(local_to_body.trans_vec, msg->pos, 3*sizeof(double));
  memcpy(local_to_body.rot_quat, msg->orientation, 4*sizeof(double));
  
  init_state.utime = msg->utime;

  init_state.position() = Eigen::Map<const Eigen::Vector3d>(local_to_body.trans_vec);
  eigen_utils::botDoubleToQuaternion(init_state.orientation(), local_to_body.rot_quat);

  init_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = cov_pose_meas.topLeftCorner<3, 3>();
  init_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = cov_pose_meas.bottomRightCorner<3, 3>();

  Vector3d init_rpy_deg = bot_to_degrees(init_state.getEulerAngles());

  fprintf(stderr, "initialized position using a pose_t at xyz: %f,%f,%f\n", init_state.position()(0),
      init_state.position()(1), init_state.position()(2));
  fprintf(stderr, "initialized orientation using a pose_t at rpy: %f,%f,%f\n", init_rpy_deg(0),
      init_rpy_deg(1), init_rpy_deg(2));

  return true;
}

}

