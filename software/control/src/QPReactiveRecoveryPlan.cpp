#include "QPReactiveRecoveryPlan.hpp"
#include <unsupported/Eigen/Polynomials>
extern "C" {
  #include "iris_ldp/solver.h"
}

#define CVXGEN_MAX_ROWS 3
#define CVXGEN_MAX_PTS 8

Vars vars;
Params params;
Workspace work;
Settings settings;

VectorXd QPReactiveRecoveryPlan::closestPointInConvexHull(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V) {

  int dim = x.size();

  Matrix<double, CVXGEN_MAX_ROWS, 1> x_resized;
  Matrix<double, CVXGEN_MAX_ROWS, CVXGEN_MAX_PTS> V_resized;

  if (dim > CVXGEN_MAX_ROWS) {
    fprintf(stderr, "x can not be more than %d elements\n", CVXGEN_MAX_ROWS);
    exit(1);
  }
  if (V.rows() != dim) {
    fprintf(stderr, "V must have same number of rows as x\n");
    exit(1);
  }
  if (V.cols() > CVXGEN_MAX_PTS) {
    fprintf(stderr, "V can not be larger than %d x %d\n", CVXGEN_MAX_ROWS, CVXGEN_MAX_PTS);
    exit(1);
  }

  x_resized.head(dim) = x;
  V_resized.block(0, 0, dim, V.cols()) = V;

  if (dim < CVXGEN_MAX_ROWS) {
    x_resized.tail(CVXGEN_MAX_ROWS - dim) = VectorXd::Zero(CVXGEN_MAX_ROWS-dim);
    V_resized.block(dim, 0, CVXGEN_MAX_ROWS - dim, V_resized.cols()) = MatrixXd::Zero(CVXGEN_MAX_ROWS-dim, V_resized.cols());
  }

  for (int i=V.cols(); i < CVXGEN_MAX_PTS; i++) {
    for (int j=0; j < CVXGEN_MAX_ROWS; j++) {
      V_resized(j,i) = V_resized(j,i-1);
    }
  }

  for (int j=0; j < CVXGEN_MAX_PTS; j++) {
    V_resized.col(j) = V_resized.col(j) - x_resized;
  }

  set_defaults();
  setup_indexing();
  settings.verbose = 0;


  double *src = V_resized.data();
  double *dest = params.Y;
  for (int i=0; i < CVXGEN_MAX_ROWS * CVXGEN_MAX_PTS; i++) {
    *dest++ = *src++;
  }

  solve();


  Map<VectorXd>y(vars.v, CVXGEN_MAX_ROWS);
  Map<VectorXd>w(vars.w, CVXGEN_MAX_PTS);

  y = y + x.head(dim);
  return y.head(dim);
}

std::set<double> QPReactiveRecoveryPlan::expIntercept(const ExponentialForm &expform, double l0, double ld0, double u, int degree) {
  // Find the t >= 0 solutions to a*e^(b*t) + c == l0 + 1/2*ld0*t + 1/4*u*t^2 - 1/4*ld0^2/u
  // using a taylor expansion up to power [degree]

  Polynomial p = expform.taylorExpand(degree);
  VectorXd coefs_int_neg = VectorXd::Zero(degree+1);
  coefs_int_neg(0) = -(l0 - 0.25*ld0*ld0/u);
  coefs_int_neg(1) = -0.5*ld0;
  coefs_int_neg(2) = -0.25*u;

  Polynomial p_int = p + Polynomial(coefs_int_neg);

  PolynomialSolver<double, Dynamic> poly_solver(p_int.getCoefficients());

  std::vector<double> roots;
  poly_solver.realRoots(roots);

  // for (std::vector<double>::iterator it = roots.begin(); it != roots.end(); ++it) {
  //   std::cout << "root: " << *it << std::endl;
  //   std::cout << "value: " << p_int.value(*it) << std::endl;
  //   std::cout << "exp poly: " << p.value(*it) << std::endl;
  //   std::cout << "int neg: " << Polynomial(coefs_int_neg).value(*it) << std::endl;
  // }

  std::set<double> nonneg_roots(roots.begin(), roots.end());

  for (std::set<double>::iterator it = nonneg_roots.begin(); it != nonneg_roots.end(); ++it) {
    if (*it < 0) {
      // std::cout << "erasing: " << *it << std::endl;
      nonneg_roots.erase(it);
    } 
    // else {
    //   std::cout << "keeping: " << *it << std::endl;
    // }
  }

  return nonneg_roots;
}

