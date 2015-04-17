#include "QPReactiveRecoveryPlan.hpp"
extern "C" {
  #include "iris_ldp/solver.h"
}
#include "gurobi_c++.h"

#define CVXGEN_MAX_ROWS 3
#define CVXGEN_MAX_PTS 8

Vars vars;
Params params;
Workspace work;
Settings settings;

// #define DEBUG

VectorXd QPReactiveRecoveryPlan::closestPointInConvexHull(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V) {
  try {
    GRBEnv env = GRBEnv();
    env.set(GRB_IntParam_OutputFlag, 0);
    GRBModel model = GRBModel(env);

    #ifdef DEBUG
      std::cout << "V: " << V << std::endl;
    #endif

    int dim = x.size();
    int n_points = V.cols();

    GRBVar *y = model.addVars(dim);

    VectorXd lb = VectorXd::Zero(n_points);
    VectorXd ub = VectorXd::Zero(n_points).array() + 1;

    GRBVar *w = model.addVars(lb.data(), ub.data(), NULL, NULL, NULL, n_points);

    model.update();

    GRBQuadExpr obj;
    obj = 0;

    for (int i=0; i < dim; i++) {
      obj += (y[i] - x(i)) * (y[i] - x(i));

      GRBLinExpr sum;
      sum = 0;
      for (int j=0; j < n_points; j++) {
        sum += V(i,j) * w[j];
      }
      model.addConstr(sum == y[i]);
    }
    model.setObjective(obj);

    GRBLinExpr sum;
    sum = 0;
    for (int j=0; j < n_points; j++) {
      sum += w[j];
    }
    model.addConstr(sum == 1);

    model.optimize();

    VectorXd result = VectorXd::Zero(dim);
    VectorXd w_out = VectorXd::Zero(n_points);
    for (int i=0; i < dim; i++) {
      result(i) = y[i].get(GRB_DoubleAttr_X);
    }

    #ifdef DEBUG
      for (int j=0; j < n_points; j++) {
        w_out(j) = w[j].get(GRB_DoubleAttr_X);
      }

      std::cout << "y: " << result << "\nw: " << w_out << std::endl;
    #endif

    delete[] y;
    delete[] w;

    return result;
  } catch (GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
    exit(1);
  } 
}

VectorXd QPReactiveRecoveryPlan::closestPointInConvexHullCVXGEN(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V) {

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
