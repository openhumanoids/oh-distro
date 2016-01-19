#include "mex.h"
#include "../../src/QPReactiveRecoveryPlan.hpp"
#include "drake/util/drakeMexUtil.h"

using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs == 1) {
    // zero-input argument form allows us to make sure the mex file is loaded, without actually doing anything
    return;
  }

  if (nrhs != 4 || nlhs > 1) mexErrMsgTxt("usage: msg_data = getQPControllerInput(obj, t_global, x, contact_force_detected)");

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

  drake::lcmt_qp_controller_input msg = plan->getQPControllerInput(t_global, q, v, contact_force_detected);
  if (nlhs > 0) {
    int msg_size = msg.getEncodedSize();
    plhs[0] = mxCreateNumericMatrix(msg_size, 1, mxUINT8_CLASS, mxREAL);
    void *msg_ptr = mxGetData(plhs[0]);
    int num_written = msg.encode(msg_ptr, 0, msg_size);
    if (num_written != msg_size) {
      mexPrintf("Number written: %d, message size: %d\n", num_written, msg_size);
      mexErrMsgTxt("Number of bytes written does not match message size.\n");
    }
  }

}
