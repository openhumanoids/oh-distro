#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 4 || nlhs != 1) mexErrMsgTxt("usage: x_ic_new=icpUpdate(x_ic, x_cop, dt, omega)");

  sizecheck(prhs[0], 1, 1);
  double x_ic = mxGetScalar(prhs[0]);
  sizecheck(prhs[1], 1, 1);
  double x_cop = mxGetScalar(prhs[1]);
  sizecheck(prhs[2], 1, 1);
  double dt = mxGetScalar(prhs[2]);
  sizecheck(prhs[3], 1, 1);
  double omega = mxGetScalar(prhs[3]);

  ExponentialForm expform((x_ic - x_cop), omega, x_cop);
  VectorXd x_ic_new(1);
  x_ic_new(0) = expform.value(dt);
  plhs[0] = eigenToMatlab(x_ic_new);
}
