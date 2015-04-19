#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 7 || (nlhs != 1 && nlhs != 2)) mexErrMsgTxt("usage: [t_int, x_int]=expIntercept(a, b, c, l0, ld0, u, degree)");

  sizecheck(prhs[0], 1, 1);
  double a = mxGetScalar(prhs[0]);
  sizecheck(prhs[1], 1, 1);
  double b = mxGetScalar(prhs[1]);
  sizecheck(prhs[2], 1, 1);
  double c = mxGetScalar(prhs[2]);
  sizecheck(prhs[3], 1, 1);
  double l0 = mxGetScalar(prhs[3]);
  sizecheck(prhs[4], 1, 1);
  double ld0 = mxGetScalar(prhs[4]);
  sizecheck(prhs[5], 1, 1);
  double u = mxGetScalar(prhs[5]);
  sizecheck(prhs[6], 1, 1);
  int degree = static_cast<int> (mxGetScalar(prhs[6]));

  ExponentialForm expform(a, b, c);
  std::vector<double> roots = QPReactiveRecoveryPlan::expIntercept(expform, l0, ld0, u, degree);


  VectorXd t_int(roots.size());
  VectorXd x_int(roots.size());
  int i=0;
  for (std::vector<double>::iterator it = roots.begin(); it != roots.end(); ++it) {
    t_int(i) = *it;
  }
  plhs[0] = eigenToMatlab(t_int.transpose());

  if (nlhs == 2) {
    VectorXd coefs_int(3);
    coefs_int << l0 - 0.25*ld0*ld0/u, 0.5*ld0, 0.25*u;
    Polynomial p_int(coefs_int);
    for (int j=0; j < t_int.size(); j++) {
      x_int(j) = p_int.value(t_int(j));
    }
    plhs[1] = eigenToMatlab(x_int.transpose());
  }
}
