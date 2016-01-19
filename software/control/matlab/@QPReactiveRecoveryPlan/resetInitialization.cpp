#include "mex.h"
#include "../../src/QPReactiveRecoveryPlan.hpp"
#include "drake/util/drakeMexUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 1 || nlhs != 0) mexErrMsgTxt("usage: resetInitialization(obj)");

  int narg = 0;
  sizecheck(prhs[narg], 1, 1);
  QPReactiveRecoveryPlan *plan = (QPReactiveRecoveryPlan*) getDrakeMexPointer(mxGetPropertySafe(prhs[narg], "mex_ptr"));
  ++narg;

  plan->resetInitialization();

}
