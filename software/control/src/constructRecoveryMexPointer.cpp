#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"
#include "drake/QPCommon.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs == 1) {
    // by convenction, calling with one argument should free the pointer
    if (isa(prhs[0], "DrakeMexPointer")) {
      destroyDrakeMexPointer<QPReactiveRecoveryPlan*>(prhs[0]);
      return;
    } else {
      mexErrMsgIdAndTxt("Drake:QPReactiveRecoveryPlan:constructRecoveryMexPointer:BadInputs", "Expected a DrakeMexPointer (or a subclass)");
    }
  }

  if (nrhs != 3 || nlhs != 1) mexErrMsgTxt("usage: mex_ptr = constructMexPointer(robot_ptr, qstar, S_lyapunov)");

  int narg = 0;
  sizecheck(prhs[narg], 1, 1);
  QPReactiveRecoveryPlan *plan;
  plan = new QPReactiveRecoveryPlan((RigidBodyManipulator*) getDrakeMexPointer(prhs[narg]));
  ++narg;

  sizecheck(prhs[narg], plan->robot->num_positions, 1);
  Map<VectorXd> qstar(mxGetPrSafe(prhs[narg]), mxGetNumberOfElements(prhs[narg]));
  plan->setQDes(qstar);
  ++narg;

  sizecheck(prhs[narg], 4, 4);
  Map<Matrix4d>S(mxGetPrSafe(prhs[narg]));
  plan->setS(S);
  ++narg;

  plhs[0] = createDrakeMexPointer((void*) plan, "QPReactiveRecoveryPlan");
}