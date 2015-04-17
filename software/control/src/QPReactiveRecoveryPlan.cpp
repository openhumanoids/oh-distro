#include "QPReactiveRecoveryPlan.hpp"
#include "gurobi_c++.h"

// #define DEBUG

VectorXd QPReactiveRecoveryPlan::closestPointInConvexHull(VectorXd &x, MatrixXd &V) {
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

