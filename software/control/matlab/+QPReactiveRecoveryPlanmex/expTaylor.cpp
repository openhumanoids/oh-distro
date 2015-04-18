#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 4 || nlhs != 1) mexErrMsgTxt("usage: p=expTaylor(a, b, c, degree)");

  sizecheck(prhs[0], 1, 1);
  sizecheck(prhs[1], 1, 1);
  sizecheck(prhs[2], 1, 1);
  sizecheck(prhs[3], 1, 1);

  double a = mxGetScalar(prhs[0]);
  double b = mxGetScalar(prhs[1]);
  double c = mxGetScalar(prhs[2]);
  int degree = static_cast<int> (mxGetScalar(prhs[3]));
  Polynomial p = ExponentialForm(a, b, c).taylorExpand(degree);
  VectorXd coefs = p.getCoefficients();
  coefs.reverseInPlace();
  plhs[0] = eigenToMatlab(coefs);
}
