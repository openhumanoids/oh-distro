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

Polynomial QPReactiveRecoveryPlan::bangBangPolynomial(double x0, double xd0, double u) {
  VectorXd coefs(3);
  coefs << x0 - 0.25*xd0*xd0/u,
           0.5*xd0,
           0.25*u;
  Polynomial p(coefs);
  return p;
}

std::vector<double> realRoots(Polynomial p) {
  VectorXd coefs = p.getCoefficients();
  double order = p.getOrder();
  std::vector<double> roots;
  if (order == 1) {
    // c0 + c1*t = 0;
    // t = -c0/c1
    roots.push_back(-coefs(0) / coefs(1));
  } else if (order == 2) {
    // c0 + c1*t + c2*t^2 = 0;
    // t = (-c1 +- sqrt(c1^2 - 4*c2*c0)) / (2*c2)
    double discriminant = pow(coefs(1), 2) - 4*coefs(2)*coefs(0);
    if (discriminant >= 0) {
      roots.push_back((-coefs(1) + sqrt(discriminant)) / (2*coefs(2)));
      roots.push_back((-coefs(1) - sqrt(discriminant)) / (2*coefs(2)));
    }
  } else {
    PolynomialSolver<double, Dynamic> poly_solver(coefs);
    poly_solver.realRoots(roots);
  }
  return roots;
}

std::vector<double> QPReactiveRecoveryPlan::expIntercept(const ExponentialForm &expform, double l0, double ld0, double u, int degree) {
  // Find the t >= 0 solutions to a*e^(b*t) + c == l0 + 1/2*ld0*t + 1/4*u*t^2 - 1/4*ld0^2/u
  // using a taylor expansion up to power [degree]

  Polynomial p_taylor = expform.taylorExpand(degree);
  VectorXd coefs = -QPReactiveRecoveryPlan::bangBangPolynomial(l0, ld0, u).getCoefficients();
  coefs.conservativeResize(6);
  coefs.tail(3).setZero();
  Polynomial p_bang(coefs);
  Polynomial p_int = p_taylor + p_bang;

  std::vector<double> roots = realRoots(p_int);
  std::vector<double> nonneg_roots;

  for (std::vector<double>::iterator it = roots.begin(); it != roots.end(); ++it) {
    if (*it > 0) {
      nonneg_roots.push_back(*it);
    } 
  }

  return nonneg_roots;
}

std::vector<BangBangIntercept> QPReactiveRecoveryPlan::bangBangIntercept(double x0, double xd0, double xf, double u_max) {
  std::vector<BangBangIntercept> intercepts;

  double us[2] = {u_max, -u_max};
  for (int i=0; i<2; i++) {
    double u = us[i];
    Polynomial p = QPReactiveRecoveryPlan::bangBangPolynomial(x0 - xf, xd0, u);
    std::vector<double> roots = realRoots(p);
    for (std::vector<double>::iterator it = roots.begin(); it != roots.end(); ++it) {
      double t = *it;
      if (t >= std::abs(xd0 / u)) {
        BangBangIntercept inter;
        inter.tf = t;
        inter.tswitch = 0.5 * (t - xd0 / u);
        inter.u = u;
        intercepts.push_back(inter);
        break;
      }
    }
  }
  return intercepts;
}


