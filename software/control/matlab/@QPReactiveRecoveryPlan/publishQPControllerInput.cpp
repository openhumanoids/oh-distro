#include "mex.h"
#include "../../src/QPReactiveRecoveryPlan.hpp"
#include "drake/util/drakeMexUtil.h"

using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs == 1) {
    // zero-input argument form allows us to make sure the mex file is loaded, without actually doing anything
    return;
  }

  if (nrhs != 4 || nlhs != 0) mexErrMsgTxt("usage: publishQPControllerInput(obj, t_global, x, contact_force_detected)");

  int narg = 0;
  sizecheck(prhs[narg], 1, 1);
  QPReactiveRecoveryPlan *plan = (QPReactiveRecoveryPlan*) getDrakeMexPointer(mxGetPropertySafe(prhs[narg], "mex_ptr"));
  ++narg;

  sizecheck(prhs[narg], 1, 1);
  double t_global = mxGetScalar(prhs[narg]);
  ++narg;

  sizecheck(prhs[narg], plan->robot->num_positions + plan->robot->num_velocities, 1);
  Map<VectorXd> x(mxGetPrSafe(prhs[narg]), mxGetNumberOfElements(prhs[narg]));
  VectorXd q = x.head(plan->robot->num_positions);
  VectorXd v = x.tail(plan->robot->num_velocities);
  ++narg;

  
  sizecheck(prhs[narg], plan->robot->bodies.size(), 1);
  Map<VectorXd> contact_force_detected_double(mxGetPrSafe(prhs[narg]), mxGetNumberOfElements(prhs[narg]));
  std::vector<bool> contact_force_detected;
  contact_force_detected.resize(plan->robot->bodies.size());
  for (int i=0; i < contact_force_detected_double.size(); ++i) {
    contact_force_detected[i] = contact_force_detected_double[i] > 0.5;
  }
  ++narg;

  plan->publishQPControllerInput(t_global, q, v, contact_force_detected);
}
