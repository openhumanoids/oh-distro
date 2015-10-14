//
// Created by manuelli on 10/13/15.
//

#include <gurobi_c++.h>
#include <Eigen/Dense>
#include <stdexcept>
#define NDEBUG

namespace{

using namespace Eigen;

void gurobiAddQuadraticObjective(GRBModel &model, const MatrixXd &Q, std::vector<GRBVar> vars) {
  int n = Q.cols();
  assert(Q.rows() == n); //enforce that this be a square matrix
  assert(vars.size() == n); //size of Q and number of vars must match

  GRBQuadExpr quadObjective = model.getObjective();


  for (int i=0; i < n; i++){
    for (int j=0; j < n; j++){
      quadObjective.addTerm(Q(i,j), vars[i], vars[j]);
    }
  }

  model.setObjective(quadObjective);
  model.update();


}

void gurobiAddLinearObjective(GRBModel &model, const VectorXd &f, std::vector<GRBVar> vars) {
  int n = vars.size();
//  std::cout << "number of vars is " << n << std::endl;
//  std::cout << "size of f is " << f.size() << std::endl;
  if (f.size()!=n){
   throw std::invalid_argument("f doesn't match size of number of variables");
  } // f and vars must be the same size

  GRBQuadExpr objective = model.getObjective();
  for (int i=0; i < n; i++){
    objective.addTerm(f(i),vars[i]);
  }

  model.setObjective(objective);
  model.update();

}

void gurobiAddObjective(GRBModel &model, const MatrixXd &Q, const VectorXd &f, std::vector<GRBVar> vars){
  gurobiAddQuadraticObjective(model, Q, vars);
  gurobiAddLinearObjective(model, f, vars);
}

};

