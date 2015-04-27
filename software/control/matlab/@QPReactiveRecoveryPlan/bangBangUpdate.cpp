#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 4 || nlhs != 1) mexErrMsgTxt("usage: xf=bangBangUpdate(x0, xd0, tf, u)");

  sizecheck(prhs[0], 1, 1);
  double x0 = mxGetScalar(prhs[0]);
  sizecheck(prhs[1], 1, 1);
  double xd0 = mxGetScalar(prhs[1]);
  sizecheck(prhs[2], 1, 1);
  double tf = mxGetScalar(prhs[2]);
  sizecheck(prhs[3], 1, 1);
  double u = mxGetScalar(prhs[3]);

  Polynomial<double> p = QPReactiveRecoveryPlan::bangBangPolynomial(x0, xd0, u);
  VectorXd xf(1);
  xf(0) = p.value(tf);
  
  plhs[0] = eigenToMatlab(xf);
}
