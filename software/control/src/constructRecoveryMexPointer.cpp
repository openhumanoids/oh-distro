#include "mex.h"
#include "QPReactiveRecoveryPlan.hpp"
#include "drake/util/drakeMexUtil.h"
#include "drake/systems/controllers/QPCommon.h"
#include "drake/systems/controllers/controlUtil.h"
#include "drake/util/yaml/yamlUtil.h"

using namespace Eigen;

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

  if (nrhs != 4 || nlhs != 1) mexErrMsgTxt("usage: mex_ptr = constructMexPointer(robot_ptr, qstar, S_lyapunov, control_config_filename)");

  int narg = 0;
  sizecheck(prhs[narg], 1, 1);
  RigidBodyTree* model = (RigidBodyTree*) getDrakeMexPointer(prhs[narg]);
  ++narg;

  sizecheck(prhs[narg], model->num_positions, 1);
  Map<VectorXd> qstar(mxGetPrSafe(prhs[narg]), mxGetNumberOfElements(prhs[narg]));
  ++narg;

  sizecheck(prhs[narg], 4, 4);
  Map<Matrix4d>S(mxGetPrSafe(prhs[narg]));
  ++narg;

  YAML::Node control_config = YAML::LoadFile(mxGetStdString(prhs[narg]));
  RobotPropertyCache rpc = parseKinematicTreeMetadata(control_config["kinematic_tree_metadata"],
                                   *model);
  ++narg;

  QPReactiveRecoveryPlan *plan;
  plan = new QPReactiveRecoveryPlan(model, rpc);
  plan->setQDes(qstar);
  plan->setS(S);

  plhs[0] = createDrakeMexPointer((void*) plan, "QPReactiveRecoveryPlan");
}
