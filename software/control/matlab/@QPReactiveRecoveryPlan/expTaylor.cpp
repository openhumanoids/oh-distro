#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 4 || nlhs != 1) mexErrMsgTxt("usage: p=expTaylor(a, b, c, degree)");

  int M = mxGetM(prhs[0]);
  int N = mxGetN(prhs[0]);
  sizecheck(prhs[1], M, N);
  sizecheck(prhs[2], M, N);
  sizecheck(prhs[3], 1, 1);

  Map<VectorXd> a(mxGetPrSafe(prhs[0]), M*N);
  Map<VectorXd> b(mxGetPrSafe(prhs[1]), M*N);
  Map<VectorXd> c(mxGetPrSafe(prhs[2]), M*N);

  int degree = static_cast<int> (mxGetScalar(prhs[3]));

  MatrixXd all_coefs(degree+1, a.size());

  for (int i=0; i < a.size(); i++) {
    Polynomial p = ExponentialForm(a(i), b(i), c(i)).taylorExpand(degree);
    VectorXd coefs = p.getCoefficients();
    all_coefs.col(i) = coefs.reverse();
  }
  plhs[0] = eigenToMatlab(all_coefs);
}
