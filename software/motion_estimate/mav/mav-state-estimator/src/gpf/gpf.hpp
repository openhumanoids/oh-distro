#ifndef __gpf_h__
#define __gpf_h__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_utils/eigen_utils.hpp>
#include <lcmtypes/mav_indexed_measurement_t.h>
#include <lcmtypes/mav/indexed_measurement_t.hpp>
#include <bot_lcmgl_client/lcmgl.h>

#include <mav_state_est/rbis.hpp>

namespace MavStateEst {

#define GPF_R_NEG_EIG_CORRECTION 10000.0 //make this large since it means we have no information along this axis
/**
 * static polymorphism:
 *
 * http://en.wikipedia.org/wiki/Template_metaprogramming#Static_polymorphism
 */
template<class LikelihoodInterface>
class GPFLikelihoodInterface {
public:
  //returns POSITIVE log likelihood of the state given the measurement
  double likelihoodFunction(const RBIS & state)
  {
    return static_cast<LikelihoodInterface*>(this)->likelihoodFunction(state);
  }
};

template<class LikelihoodInterface>
void gpfMeasurement(GPFLikelihoodInterface<LikelihoodInterface> * likelihood_interface, const RBIS & state,
    const RBIM & cov,
    const Eigen::VectorXi & z_indices, Eigen::VectorXd & z_effective, Eigen::MatrixXd & R_effective, int num_samples,
    double max_weight_proportion,
    bot_lcmgl_t * lcmgl = NULL);

mav_indexed_measurement_t * gpfCreateLCMmsg(const Eigen::VectorXi & z_indices, const Eigen::VectorXd & z_effective,
    const Eigen::MatrixXd & R_effective);

mav::indexed_measurement_t * gpfCreateLCMmsgCPP(const Eigen::VectorXi & z_indices, const Eigen::VectorXd & z_effective,
    const Eigen::MatrixXd & R_effective);

template<class LikelihoodInterface>
void gpfMeasurement(GPFLikelihoodInterface<LikelihoodInterface> * likelihood_interface, const RBIS & state,
    const RBIM & cov,
    const Eigen::VectorXi & z_indices, Eigen::VectorXd & z_effective, Eigen::MatrixXd & R_effective, int num_samples,
    double max_weight_proportion,    
    bot_lcmgl_t * lcmgl = NULL)
{
  using namespace Eigen;
  using namespace eigen_utils;

  //"measurement vector" is [delta]

  int m = z_indices.rows();

  //get the marginal distribution for part of the state measured
  MatrixXd Sigma_bar(m, m);

  for (int ii = 0; ii < m; ii++) {
    for (int jj = 0; jj < m; jj++) {
      Sigma_bar(ii, jj) = cov(z_indices(ii), z_indices(jj));
    }
  }

//  eigen_dump(z_indices);
//  eigen_dump(Sigma);
//  eigen_dump(cov);

  MatrixXd Sigma_bar_choldecomp(m, m);
  Sigma_bar_choldecomp = Sigma_bar.llt().matrixL(); //cholesky decomposition for adjusting identity samples

  MatrixXd Sigma_bar_samples(m, num_samples);
  ArrayXd weights(num_samples);

  //loop variables
  VectorXd samp(m);
  RBIS dstate_samp;
  RBIS samp_state;

  for (int ii = 0; ii < num_samples; ii++) {
    randn_identity(samp);
    samp = Sigma_bar_choldecomp * samp;
    Sigma_bar_samples.col(ii) = samp;

    //add the sample to the state
    dstate_samp = RBIS();

    for (int k = 0; k < m; k++) {
      dstate_samp.vec(z_indices(k)) = samp(k);
    }
    samp_state = state;
    samp_state.addState(dstate_samp);

    //get the log likelihood
    weights(ii) = likelihood_interface->likelihoodFunction(samp_state);

  }

  double max_weight = weights.maxCoeff();
  weights = (weights - max_weight).exp();

  double weights_sum = weights.sum();
  const double min_weight_sum = m * 5;
  // max_weight_proportion disallows small corrections from being incorporated. it was .99, now .999 mfallon april 2014
  const double max_weight_sum = max_weight_proportion * num_samples; 

  if (min_weight_sum < weights_sum && weights_sum < max_weight_sum) {

    //compute the mean and covariance of the unweighted sample sistribution
    VectorXd Sigma_bar_sample_mean(m);
    MatrixXd Sigma_bar_sample_cov(m, m);
    ArrayXd uniform_weights(num_samples);
    uniform_weights = ArrayXd::Ones(num_samples, 1);
    fitParticles(Sigma_bar_samples, uniform_weights, Sigma_bar_sample_mean, Sigma_bar_sample_cov);

//  eigen_dump(Sigma_bar_sample_mean);
//  eigen_dump(Sigma_bar_sample_cov);

    VectorXd Sigma_sample_mean(m);
    MatrixXd Sigma_sample_cov(m, m);
    fitParticles(Sigma_bar_samples, weights, Sigma_sample_mean, Sigma_sample_cov);

//  eigen_dump(Sigma_sample_mean);
//  eigen_dump(Sigma_bar_sample_cov);

    //solve for the effective measurement noise ASSUMING Identity observation function on Delta
    MatrixXd R_effective_inverse(m, m);
    R_effective_inverse = Sigma_sample_cov.inverse() - Sigma_bar_sample_cov.inverse();
    R_effective.resize(m, m);
    R_effective = R_effective_inverse.inverse();
    MatrixXd S_effective(m, m);
    S_effective = Sigma_bar + R_effective;
    //  K = self->Sigma * C.transpose() * S.inverse();
    MatrixXd K_effective(m, m);
    K_effective.transpose() = S_effective.ldlt().solve(Sigma_bar_sample_cov);

    //solve for effective state update ASSUMING the same
    VectorXd z_resid_effective(m);
    VectorXd x_measured(m);

    for (int k = 0; k < m; k++) {
      x_measured(k) = state.vec(z_indices(k));
    }

    z_resid_effective = K_effective.colPivHouseholderQr().solve(Sigma_sample_mean - Sigma_bar_sample_mean);

    z_effective.resize(m);
    z_effective = x_measured + z_resid_effective; // A posteriori update of relevent dimensions of the state

    //  eigen_dump(R_effective);

    //Fix ill conditioned covariance matrices
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(R_effective);
    Eigen::VectorXd eig_vals = eigen_solver.eigenvalues();
    Eigen::MatrixXd eig_vecs = eigen_solver.eigenvectors();

    if ((eig_vals.array() < 0).any()) {
      bool verbose = false;
      if (verbose) {
        std::cout << "warning, R matrix from laser gpf measurement has negative eigen values, fixing\n";
        eigen_dump(R_effective);
        eigen_dump(eig_vals);
        eigen_dump(eig_vecs);
      }

      for (int ii = 0; ii < m; ii++) {
        if (eig_vals(ii) < 0) {
          eig_vals(ii) = GPF_R_NEG_EIG_CORRECTION;
        }
      }

      R_effective = eig_vecs * eig_vals.asDiagonal() * eig_vecs.inverse();
      if (verbose) {
        std::cout << "fixed to:\n";
        eigen_dump(R_effective);
        eigen_dump(eig_vals);
        eigen_dump(eig_vecs);
      }
    }

  }
  else {
    fprintf(stderr, "WARNING: weights are ill-conditioned. weight_sum=%f not in [%f, %f]\n", weights_sum,
        min_weight_sum, max_weight_sum);
    R_effective = GPF_R_NEG_EIG_CORRECTION * MatrixXd::Identity(m, m);
    z_effective.resize(m);
    for (int k = 0; k < m; k++) {
      z_effective(k) = state.vec(z_indices(k));
    }
  }

  if (lcmgl != NULL) {
    double max_weight = weights(0);
    double min_weight = weights(0);
    Vector3d delta_samp;

    for (int ii = 0; ii < num_samples; ii++) {
      if (weights[ii] > max_weight)
        max_weight = weights(ii);
      if (weights[ii] < min_weight)
        min_weight = weights(ii);
    }

    bot_lcmgl_enable(lcmgl, GL_DEPTH_TEST);
    bot_lcmgl_point_size(lcmgl, 4);
    bot_lcmgl_begin(lcmgl, GL_POINTS);

    double weight_range = max_weight - min_weight;
    for (int ii = 0; ii < num_samples; ii++) {
      double weight_norm = (weights(ii) - min_weight) / weight_range;
      float * color = bot_color_util_jet(weight_norm);
      bot_lcmgl_color3f(lcmgl, color[0], color[1], color[2]);

      samp = Sigma_bar_samples.col(ii);

      //add the sample to the state
      dstate_samp = RBIS();
      for (int k = 0; k < m; k++) {
        dstate_samp.vec(z_indices(k)) = samp(k);
      }
      samp_state = state;
      samp_state.addState(dstate_samp);
      delta_samp = samp_state.position();

      //      eigen_matlab_dump(samp_state.position() - state.position());

      bot_lcmgl_vertex3d(lcmgl, delta_samp(0), delta_samp(1), delta_samp(2));
    }
    bot_lcmgl_end(lcmgl);

//    Matrix2d xy_cov = Sigma_sample_cov.block<2, 2>(0, 0);
//    Vector3d xyz_cov_center = Sigma_sample_mean + state.Delta();
//    bot_lcmgl_color3f(lcmgl, 0, 1, 0);
//    lcmglLineWidth(3);
//    bot_lcmgl_cov_ellipse(lcmgl, xy_cov, xyz_cov_center, 3, false);

    bot_lcmgl_disable(lcmgl, GL_DEPTH_TEST);
//    bot_lcmgl_switch_buffer(lcmgl);
  }

}

}
#endif
