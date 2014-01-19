#include "gpf.hpp"

using namespace eigen_utils;
using namespace Eigen;
using namespace std;

namespace MavStateEst {

mav_indexed_measurement_t * gpfCreateLCMmsg(const Eigen::VectorXi & z_indices, const Eigen::VectorXd & z_effective,
    const Eigen::MatrixXd & R_effective)
{
  mav_indexed_measurement_t * msg = (mav_indexed_measurement_t *) calloc(1, sizeof(mav_indexed_measurement_t));

  int n = z_indices.rows();
  msg->measured_dim = n;
  msg->measured_cov_dim = n * n;
  msg->R_effective = (double *) calloc(msg->measured_cov_dim, sizeof(double));
  msg->z_effective = (double *) calloc(msg->measured_dim, sizeof(double));
  msg->z_indices = (int32_t *) calloc(msg->measured_dim, sizeof(int32_t));

  memcpy(msg->R_effective, R_effective.data(), msg->measured_cov_dim * sizeof(double));
  memcpy(msg->z_effective, z_effective.data(), msg->measured_dim * sizeof(double));

  for (int ii = 0; ii < n; ii++) {
    msg->z_indices[ii] = z_indices(ii); //for safety in case we have different size ints do manual copy
  }

  return msg;
}

mav::indexed_measurement_t * gpfCreateLCMmsgCPP(const Eigen::VectorXi & z_indices, const Eigen::VectorXd & z_effective,
    const Eigen::MatrixXd & R_effective)
{
  mav::indexed_measurement_t * msg = new mav::indexed_measurement_t();

  int n = z_indices.rows();
  msg->measured_dim = n;
  msg->measured_cov_dim = n * n;
  msg->R_effective.resize(msg->measured_cov_dim);
  msg->z_effective.resize(msg->measured_dim);
  msg->z_indices.resize(msg->measured_dim);

  memcpy(&msg->R_effective[0], R_effective.data(), msg->measured_cov_dim * sizeof(double));
  memcpy(&msg->z_effective[0], z_effective.data(), msg->measured_dim * sizeof(double));

  for (int ii = 0; ii < n; ii++) {
    msg->z_indices[ii] = z_indices(ii); //for safety in case we have different size ints do manual copy
  }

  return msg;
}
}
