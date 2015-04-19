#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 4 || nlhs != 3) mexErrMsgTxt("usage: [tf, tswitch, u]=bangbangIntercept(x0, xd0, xf, u_max)");

  sizecheck(prhs[0], 1, 1);
  double x0 = mxGetScalar(prhs[0]);
  sizecheck(prhs[1], 1, 1);
  double xd0 = mxGetScalar(prhs[1]);
  sizecheck(prhs[2], 1, 1);
  double xf = mxGetScalar(prhs[2]);
  sizecheck(prhs[3], 1, 1);
  double u_max = mxGetScalar(prhs[3]);

  std::vector<BangBangIntercept> intercepts = QPReactiveRecoveryPlan::bangBangIntercept(x0, xd0, xf, u_max);

  VectorXd tf(intercepts.size());
  VectorXd tswitch(intercepts.size());
  VectorXd u(intercepts.size());

  for (std::vector<BangBangIntercept>::iterator it = intercepts.begin(); it != intercepts.end(); ++it) {
    tf(it - intercepts.begin()) = it->tf;
    tswitch(it - intercepts.begin()) = it->tswitch;
    u(it - intercepts.begin()) = it->u;
  }

  plhs[0] = eigenToMatlab(tf);
  plhs[1] = eigenToMatlab(tswitch);
  plhs[2] = eigenToMatlab(u);
}
