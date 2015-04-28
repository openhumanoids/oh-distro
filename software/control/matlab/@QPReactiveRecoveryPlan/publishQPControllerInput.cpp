#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 5 || nlhs != 0) mexErrMsgTxt("usage: publishQPControllerInput(obj, t_global, x, rpc, contact_force_detected)");

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

  
  sizecheck(prhs[narg], 1, 1);
  std::shared_ptr<RobotPropertyCache> rpc(new struct RobotPropertyCache);
  parseRobotPropertyCache(prhs[narg], rpc.get());
  ++narg;

  sizecheck(prhs[narg], plan->robot->num_bodies, 1);
  Map<VectorXd> contact_force_detected_double(mxGetPrSafe(prhs[narg]), mxGetNumberOfElements(prhs[narg]));
  std::vector<bool> contact_force_detected;
  contact_force_detected.resize(plan->robot->num_bodies);
  for (int i=0; i < contact_force_detected_double.size(); ++i) {
    contact_force_detected[i] = contact_force_detected_double[i] > 0.5;
  }
  ++narg;

  plan->publishQPControllerInput(t_global, q, v, *rpc, contact_force_detected);
}