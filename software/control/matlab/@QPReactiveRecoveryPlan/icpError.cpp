#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 4 || nlhs != 1) mexErrMsgTxt("usage: error=icpError(obj, r_ic, foot_states, foot_vertices)");

  sizecheck(prhs[0], 1, 1);
  const mxArray *obj = prhs[0];
  QPReactiveRecoveryPlan plan(NULL);
  plan.capture_max_flyfoot_height = mxGetScalar(mxGetPropertySafe(obj, "CAPTURE_MAX_FLYFOOT_HEIGHT"));
  plan.capture_shrink_factor = mxGetScalar(mxGetPropertySafe(obj, "CAPTURE_SHRINK_FACTOR"));

  sizecheck(prhs[1], 2, 1);
  Map<Vector2d> r_ic(mxGetPrSafe(prhs[1]));

  sizecheck(prhs[2], 1, 1);
  const mxArray *foot_states_obj = prhs[2];
  std::map<FootID, FootState> foot_states;

  sizecheck(prhs[3], 1, 1);
  const mxArray *foot_vertices_obj = prhs[3];
  std::map<FootID, Matrix<double, 3, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT>> foot_vertices;

  const mxArray *pobj;

  std::set<std::string> foot_names;
  foot_names.insert("right");
  foot_names.insert("left");

  for (std::set<std::string>::iterator foot_name = foot_names.begin(); foot_name != foot_names.end(); ++foot_name) {
    const mxArray *state_obj = mxGetFieldSafe(foot_states_obj, *foot_name);

    FootState state;
    state.pose.setIdentity();

    pobj = mxGetFieldSafe(state_obj, "xyz_quat");
    sizecheck(pobj, 7, 1);
    Map<XYZQuat> xyz_quat(mxGetPrSafe(pobj));
    state.pose.translate(Vector3d(xyz_quat.head(3))).rotate(Quaternion<double>(xyz_quat(3), xyz_quat(4), xyz_quat(5), xyz_quat(6)));

    pobj = mxGetFieldSafe(state_obj, "xyz_quatdot");
    sizecheck(pobj, 7, 1);
    Map<XYZQuat> xyz_quatdot(mxGetPrSafe(pobj));
    state.velocity = xyz_quatdot;

    pobj = mxGetFieldSafe(state_obj, "contact");
    sizecheck(pobj, 1, 1);
    state.contact = static_cast<bool> (mxGetScalar(pobj));

    pobj = mxGetFieldSafe(state_obj, "terrain_height");
    sizecheck(pobj, 1, 1);
    state.terrain_height = mxGetScalar(pobj);

    foot_states[footNameToID[*foot_name]] = state;

    const mxArray *vert_obj = mxGetFieldSafe(foot_vertices_obj, *foot_name);
    sizecheck(vert_obj, 2, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT);
    Map<Matrix<double, 2, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT>> V(mxGetPrSafe(vert_obj));
    foot_vertices[footNameToID[*foot_name]].topRows(2) = V;
    foot_vertices[footNameToID[*foot_name]].row(2).setZero();
  }

  Matrix<double, 1, 1> icp_error;
  icp_error(0) = plan.icpError(r_ic, foot_states, foot_vertices);
  plhs[0] = eigenToMatlab(icp_error);
}