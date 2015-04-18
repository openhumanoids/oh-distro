#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs < 2 || nlhs != 1) mexErrMsgTxt("usage: y=closestPointInConvexHull(x, V)");

  Map<VectorXd> x(mxGetPrSafe(prhs[0]), mxGetNumberOfElements(prhs[0]));
  Map<MatrixXd> V(mxGetPrSafe(prhs[1]), mxGetM(prhs[1]), mxGetN(prhs[1]));

  plhs[0] = eigenToMatlab(QPReactiveRecoveryPlan::closestPointInConvexHull(x, V));
}
