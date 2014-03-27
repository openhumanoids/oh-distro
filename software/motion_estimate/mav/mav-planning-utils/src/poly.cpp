#include "poly.hpp"

double Polynomial::eval(double t)
{
  double val = 0;
  for (int nn = 0; nn < this->coeffs.rows(); nn++) {
    val += coeffs(nn) * pow(t, nn);
  }
  return val;
}

/**
 * evaluates derivative at t
 */
double Polynomial::eval(double t, int derivative)
{
  double val = 0;
  double prod;
  for (int nn = derivative; nn < this->coeffs.rows(); nn++) {
    prod = 1;
    for (int mm = 0; mm < derivative; mm++) {
      prod *= (nn - mm);
    }
    val += coeffs(nn) * prod * pow(t, nn - derivative);
  }
  return val;
}

Polynomial Polynomial::getDerivative()
{
  Polynomial derivative(this->coeffs.rows() - 2);
  double prod;
  for (int nn = 1; nn < this->coeffs.rows(); nn++) {
    derivative.coeffs(nn - 1) = this->coeffs(nn) * nn;
  }
  return derivative;
}

std::ostream& operator<<(std::ostream& output, const Polynomial & poly)
{
  output << poly.coeffs;
  return output;
}

int sign(int v)
{
  if (v > 0)
    return 1;
  if (v < 0)
    return -1;

  return 0;
}

void polyQaudDerOptPiecewiseIndexMap(int N_extra_constraints, int D, int N_poly, int K,
    Eigen::VectorXi & index_kk_to_BR)
{

  int N_K = K * N_poly;
  int N_extra_per_K = ((double) N_extra_constraints) / ((double) K); //integer roundoff towards 0 intentionally

  int N_extra_remainder = N_extra_constraints - N_extra_per_K * K;

  int sign_remainder = sign(N_extra_remainder);
  N_extra_remainder = abs(N_extra_remainder);

  Eigen::VectorXi b_kk_sizes = Eigen::VectorXi::Constant(K, D + N_extra_per_K);
  b_kk_sizes.topRows(N_extra_remainder) = b_kk_sizes.segment(0, N_extra_remainder)
      + Eigen::VectorXi::Constant(N_extra_remainder, sign_remainder);

  Eigen::VectorXi r_kk_sizes = -b_kk_sizes + Eigen::VectorXi::Constant(K, N_poly);

  int B_start = 0;
  int B_size = b_kk_sizes.sum();
  int R_start = B_size;
  int R_size = r_kk_sizes.sum();

  index_kk_to_BR.setZero(N_K);

  int b_kk_start = 0;
  int r_kk_start = 0;

  Eigen::VectorXi x_kk_inds;
  for (int kk = 0; kk < K; kk++) {
    x_kk_inds.setLinSpaced(N_poly, kk * N_poly, (kk + 1) * N_poly - 1);
    index_kk_to_BR.segment(B_start, B_size).segment(b_kk_start, b_kk_sizes(kk)) = x_kk_inds.topRows(b_kk_sizes(kk));
    index_kk_to_BR.segment(R_start, R_size).segment(r_kk_start, r_kk_sizes(kk)) = x_kk_inds.bottomRows(r_kk_sizes(kk));
    b_kk_start += b_kk_sizes(kk);
    r_kk_start += r_kk_sizes(kk);
  }
}

void polyQuadDerOptPiecewise(const Eigen::VectorXd & taus, const Eigen::VectorXd & der_0,
    const Eigen::VectorXd & der_final, const Eigen::VectorXd & der_costs, const Eigen::MatrixXd & intermediate_der,
    Polynomial * polys[], double * cost, int intermediate_ders_fixed)
{
  int N_poly = der_costs.rows();
  int K = taus.rows();
  int D = intermediate_der.rows();

  int N_init_constraints = der_0.rows();
  int N_final_constraints = der_final.rows();
  int N_B;
  N_B = N_final_constraints + N_init_constraints + (K - 1) * (D - intermediate_ders_fixed)
      + 2 * (K - 1) * intermediate_ders_fixed; //total number of constraints
  int N_K = K * N_poly; //number of free parameters
  int N_R = N_K - N_B; //number of parameters to optimize

  if (N_R < 0) {
    fprintf(stderr,
        "Error: Not enough free parameters (polynomial degree must by higher) in polyQuadDerOptPiecewise, %s, line %d\n",
        __FILE__, __LINE__);

    for (int kk = 0; kk < K; kk++) {
      polys[kk]->coeffs.setZero(N_poly + 1);
    }
    return;
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_B, N_K);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(N_B);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(N_K, N_K);
  Eigen::MatrixXd Q_kk = Eigen::MatrixXd::Zero(N_poly, N_poly);

  for (int kk = 0; kk < K; kk++) {
    polyGetCostMatrix(taus(kk), Q_kk, der_costs);
    int kk_start = kk * N_poly;
    Q.block(kk_start, kk_start, N_poly, N_poly) = Q_kk;
  }

  int constraint_start_ind = 0;
  Eigen::MatrixXd A_final_cc = Eigen::MatrixXd::Zero(D, N_poly);
  Eigen::MatrixXd A_init_cc = Eigen::MatrixXd::Zero(D, N_poly);
  for (int cc = 0; cc <= K; cc++) { //K+1 constraint blocks
    int kk_final = cc - 1;
    int kk_init = cc;
    int final_block_ind = kk_final * N_poly;
    int init_block_ind = kk_init * N_poly;
    if (cc == 0) {
      Eigen::MatrixXd A_init_0 = Eigen::MatrixXd::Zero(N_init_constraints, N_poly);
      polyGetDerivativeMatrix(0, A_init_0);
      A.block(constraint_start_ind, init_block_ind, N_init_constraints, N_poly) = A_init_0;
      b.segment(constraint_start_ind, N_init_constraints) = der_0;
      constraint_start_ind += N_init_constraints;
    }
    else if (cc < K) {
      polyGetDerivativeMatrix(taus(kk_final), A_final_cc);
      polyGetDerivativeMatrix(0, A_init_cc);

      // First constrain the intermediate derivatives
      if (intermediate_ders_fixed != 0) {
        A.block(constraint_start_ind, final_block_ind, intermediate_ders_fixed, N_poly) = A_final_cc.topRows(
            intermediate_ders_fixed);
        b.segment(constraint_start_ind, intermediate_ders_fixed) = intermediate_der.col(cc - 1).head(
            intermediate_ders_fixed);
        constraint_start_ind += intermediate_ders_fixed;
        A.block(constraint_start_ind, init_block_ind, intermediate_ders_fixed, N_poly) = A_init_cc.topRows(
            intermediate_ders_fixed);
        b.segment(constraint_start_ind, intermediate_ders_fixed) = intermediate_der.col(cc - 1).head(
            intermediate_ders_fixed);
        constraint_start_ind += intermediate_ders_fixed;
      }

      int num_offset_constraints = D - intermediate_ders_fixed;
      A.block(constraint_start_ind, final_block_ind, num_offset_constraints, N_poly) = -A_final_cc.bottomRows(
          num_offset_constraints);
      A.block(constraint_start_ind, init_block_ind, num_offset_constraints, N_poly) = A_init_cc.bottomRows(
          num_offset_constraints);
      b.segment(constraint_start_ind, num_offset_constraints) = intermediate_der.col(cc - 1).tail(
          num_offset_constraints);
      constraint_start_ind += num_offset_constraints;

    }
    else {
      Eigen::MatrixXd A_final_K = Eigen::MatrixXd::Zero(N_final_constraints, N_poly);
      polyGetDerivativeMatrix(taus(kk_final), A_final_K);
      A.block(constraint_start_ind, final_block_ind, N_final_constraints, N_poly) = -A_final_K;
      b.segment(constraint_start_ind, N_final_constraints) = -der_final;
      constraint_start_ind += N_final_constraints;
    }
  }

  int N_extra_constraints = N_init_constraints + N_final_constraints - D;

  Eigen::VectorXi index_kk_to_BR;

  polyQaudDerOptPiecewiseIndexMap(N_extra_constraints, D, N_poly, K, index_kk_to_BR);

  Eigen::MatrixXd A_BR = Eigen::MatrixXd::Zero(N_B, N_K);
  Eigen::MatrixXd Q_BR = Eigen::MatrixXd::Zero(N_K, N_K);

  Eigen::VectorXi test_indexing(N_K);

  for (int i = 0; i < N_K; i++) {
    A_BR.col(i) = A.col(index_kk_to_BR(i));
    for (int j = 0; j < N_K; j++) {
      Q_BR(i, j) = Q(index_kk_to_BR(i), index_kk_to_BR(j));
    }
  }

  Eigen::VectorXd x_star_BR(N_K);
  Eigen::VectorXd x_star(N_K);
  eigen_utils::quadProgEliminationSolve(Q_BR, A_BR, b, x_star_BR);

  for (int i = 0; i < N_K; i++) {
    x_star(index_kk_to_BR(i)) = x_star_BR(i);
  }

  for (int kk = 0; kk < K; kk++) {

    if (taus(kk) <= .0000001) {
      x_star.segment(kk * N_poly, N_poly).setZero(); //if we have 0 length segment, 0 the polynomial
      fprintf(stderr,
          "error, 0 length segment in polyQuadDerOptPiecewise, setting coefficients to 0, may cause other problems\n");
    }
    polys[kk]->coeffs = x_star.segment(kk * N_poly, N_poly);

//    eigen_matlab_dump(*polys[kk]);
  }

  if (cost != NULL) {
    *cost = x_star.transpose() * Q * x_star;
  }
//  eigen_matlab_dump(Q);
//  eigen_matlab_dump(A);
//  eigen_matlab_dump(b);
//  eigen_matlab_dump(index_kk_to_BR);
//  eigen_matlab_dump(A_BR);
//  eigen_matlab_dump(Q_BR);
//  eigen_matlab_dump(x_star);
//  eigen_matlab_dump(x_star_BR);
}

Polynomial polyQuadDerOpt(double tau, const Eigen::VectorXd & der_0, const Eigen::VectorXd & der_final,
    const Eigen::VectorXd & der_costs, double * cost)
{

  int N_poly = der_costs.rows();
  int N_init_constraints = der_0.rows();
  int N_final_constraints = der_final.rows();
  int N_B = N_final_constraints + N_init_constraints;
  int N_R = N_poly - N_B;

  if (N_R < 0) {
    fprintf(stderr,
        "Error: Not enough free parameters (polynomial degree must by higher) in polyQuadDerOpt, %s, line %d\n",
        __FILE__, __LINE__);
    return Polynomial(N_poly - 1);
  }

  if (tau < .00000001) {
    fprintf(stderr, "error, 0 length segment in polyQuadDerOpt, setting coefficients to 0, may cause other problems\n");
    return Polynomial(N_poly - 1);
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_B, N_poly);
  Eigen::MatrixXd A_0(N_init_constraints, N_poly);
  Eigen::MatrixXd A_final(N_final_constraints, N_poly);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(N_B);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(N_poly, N_poly);

  polyGetDerivativeMatrix(0, A_0);
  polyGetDerivativeMatrix(tau, A_final);
  A.topRows(N_init_constraints) = A_0;
  A.bottomRows(N_final_constraints) = A_final;
  b.topRows(N_init_constraints) = der_0;
  b.bottomRows(N_final_constraints) = der_final;

  polyGetCostMatrix(tau, Q, der_costs);

  Polynomial poly(N_poly - 1);

  eigen_utils::quadProgEliminationSolve(Q, A, b, poly.coeffs);

  if (cost != NULL) {
    *cost = poly.coeffs.transpose() * Q * poly.coeffs;
  }

  return poly;
}

void polyGetDerivativeMatrix(double tau, Eigen::MatrixXd & A_derivative, double t_scale)
{
  int N_poly = A_derivative.cols();
  int D = A_derivative.rows();

  A_derivative.setZero();

  for (int r = 0; r < D; r++) {
    for (int n = 0; n < N_poly; n++) {
      if (n >= r) {
        double prod = 1.0;
        for (int m = 0; m < r; m++) {
          prod = prod * ((double) (n - m)) / t_scale;
        }
        A_derivative(r, n) = prod * pow(tau, n - r);
      }
    }
  }
}

/**
 * Builds a cost matrix such Q, that sum_dd der_costs(dd)*p^(dd)^T*Q*p^(dd) is the integral of p^2(t) from 0 to tau
 */
void polyGetCostMatrix(double tau, Eigen::MatrixXd & Q, const Eigen::VectorXd & der_costs)
{
  assert(der_costs.rows()==Q.rows());
  assert(der_costs.cols()==1);
  assert(Q.cols()==Q.rows());

  int N_poly = Q.cols();
  Q.setZero();

  for (int l = 0; l < N_poly; l++) {
    for (int i = 0; i < N_poly; i++) {
      for (int r = 0; r < N_poly; r++) {
        if (i >= r && l >= r) {
          double prod = 1.0;
          for (int m = 0; m < r; m++) {
            prod *= (i - m) * (l - m);
          }
          Q(i, l) += pow(der_costs(r), 2) * 2 * prod * pow(tau, l + i + 1 - 2 * r) / ((double) (l + i + 1 - 2 * r));
        }
      }
    }
  }
}

