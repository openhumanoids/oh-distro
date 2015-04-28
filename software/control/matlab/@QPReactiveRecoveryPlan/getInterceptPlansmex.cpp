#include "mex.h"
#include "control/QPReactiveRecoveryPlan.hpp"
#include "drake/drakeUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 6 || nlhs != 1) mexErrMsgTxt("usage: intercept_plans=getInterceptPlans(obj, foot_states, r_ic, comd, omega, u)");

  int narg = 0;
  sizecheck(prhs[narg], 1, 1);
  const mxArray *obj = prhs[narg];
  const mxArray *foot_vertices_obj = mxGetPropertySafe(obj, "foot_vertices");
  const mxArray *reach_verts_obj = mxGetPropertySafe(obj, "reachable_vertices");
  ++narg;

  sizecheck(prhs[narg], 1, 1);
  const mxArray *foot_states_obj = prhs[narg];
  std::map<FootID, FootState> foot_states;
  ++narg;


  sizecheck(prhs[narg], 2, 1);
  Map<Vector2d> r_ic(mxGetPrSafe(prhs[narg]));
  ++narg;

  sizecheck(prhs[narg], 3, 1);
  Map<Vector2d> comd(mxGetPrSafe(prhs[narg]));
  ++narg;

  sizecheck(prhs[narg], 1, 1);
  double omega = mxGetScalar(prhs[narg]);
  ++narg;

  sizecheck(prhs[narg], 1, 1);
  double u = mxGetScalar(prhs[narg]);
  ++narg;

  assert(narg == nrhs);

  BipedDescription biped;
  biped.u_max = u;
  biped.omega = omega;

  QPReactiveRecoveryPlan plan(NULL, biped);
  plan.capture_max_flyfoot_height = mxGetScalar(mxGetPropertySafe(obj, "CAPTURE_MAX_FLYFOOT_HEIGHT"));
  plan.capture_shrink_factor = mxGetScalar(mxGetPropertySafe(obj, "CAPTURE_SHRINK_FACTOR"));
  plan.min_step_duration = mxGetScalar(mxGetPropertySafe(obj, "MIN_STEP_DURATION"));
  plan.foot_hull_cop_shrink_factor = mxGetScalar(mxGetPropertySafe(obj, "FOOT_HULL_COP_SHRINK_FACTOR"));
  plan.max_considerable_foot_swing = mxGetScalar(mxGetPropertySafe(obj, "MAX_CONSIDERABLE_FOOT_SWING"));

  Isometry3d icp = Isometry3d(Translation<double, 3>(Vector3d(r_ic(0), r_ic(1), 0)));

  const mxArray *pobj;
  std::set<std::string> foot_names;
  foot_names.insert("right");
  foot_names.insert("left");

  for (std::set<std::string>::iterator foot_name = foot_names.begin(); foot_name != foot_names.end(); ++foot_name) {
    const mxArray *state_obj = mxGetFieldSafe(foot_states_obj, *foot_name);

    FootID foot_id = footNameToID[*foot_name];

    pobj = mxGetFieldSafe(state_obj, "xyz_quat");
    sizecheck(pobj, 7, 1);
    Map<XYZQuat> xyz_quat(mxGetPrSafe(pobj));
    foot_states[foot_id].pose = Isometry3d(Translation<double, 3>(xyz_quat.head(3)));
    foot_states[foot_id].pose.rotate(Quaternion<double>(xyz_quat(3), xyz_quat(4), xyz_quat(5), xyz_quat(6)));

    pobj = mxGetFieldSafe(state_obj, "xyz_quatdot");
    sizecheck(pobj, 7, 1);
    Map<XYZQuat> xyz_quatdot(mxGetPrSafe(pobj));
    foot_states[foot_id].velocity = xyz_quatdot;

    pobj = mxGetFieldSafe(state_obj, "contact");
    sizecheck(pobj, 1, 1);
    foot_states[foot_id].contact = static_cast<bool> (mxGetScalar(pobj));

    pobj = mxGetFieldSafe(state_obj, "terrain_height");
    sizecheck(pobj, 1, 1);
    foot_states[foot_id].terrain_height = mxGetScalar(pobj);

    pobj = mxGetFieldSafe(state_obj, "terrain_normal");
    sizecheck(pobj, 3, 1);
    Map<Vector3d> normal(mxGetPrSafe(pobj));
    foot_states[foot_id].terrain_normal = normal;

    pobj = mxGetFieldSafe(foot_vertices_obj, *foot_name);
    sizecheck(pobj, 2, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT);
    Map<Matrix<double, 2, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT>> V(mxGetPrSafe(pobj));
    biped.foot_vertices[foot_id].topRows(2) = V;
    biped.foot_vertices[foot_id].row(2).setZero();

    pobj = mxGetFieldSafe(reach_verts_obj, *foot_name);
    sizecheck(pobj, 2, 4);
    Map<Matrix<double, 2, 4>> V_reach(mxGetPrSafe(pobj));
    biped.reachable_vertices[foot_id].topRows(2) = V_reach;
    biped.reachable_vertices[foot_id].row(2).setZero();
  }

  // std::cout << "getting plans" << std::endl;
  std::vector<InterceptPlan> intercept_plans = plan.getInterceptPlans(foot_states, icp);
  // std::cout << "got plans" << std::endl;

  const size_t dims[2] = {1, intercept_plans.size()};
  const char* fields[7] = {"tf", "tswitch", "r_foot_new", "r_ic_new", "error", "swing_foot", "r_cop"};
  plhs[0] = mxCreateStructArray(2, dims, 7, fields);

  for (std::vector<InterceptPlan>::iterator p = intercept_plans.begin(); p != intercept_plans.end(); ++p) {
    int i = p - intercept_plans.begin();
    Matrix<double, 1, 1> tf;
    tf << p->tf;
    mxSetField(plhs[0], i, "tf", eigenToMatlab(tf));

    Matrix<double, 1, 1> tswitch;
    tswitch << p->tswitch;
    mxSetField(plhs[0], i, "tswitch", eigenToMatlab(tswitch));

    Matrix<double, 7, 1> r_foot_new;
    r_foot_new.head(3) = p->pose_next.translation();
    Quaternion<double> quat = Quaternion<double>(p->pose_next.rotation());
    r_foot_new.tail(4) = Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
    mxSetField(plhs[0], i, "r_foot_new", eigenToMatlab(r_foot_new));

    Matrix<double, 2, 1> r_ic_new;
    r_ic_new = p->icp_next.translation().head(2);
    mxSetField(plhs[0], i, "r_ic_new", eigenToMatlab(r_ic_new));

    Matrix<double, 1, 1> error;
    error << p->error;
    mxSetField(plhs[0], i, "error", eigenToMatlab(error));

    if (p->swing_foot == RIGHT) {
      mxSetField(plhs[0], i, "swing_foot", mxCreateString("right"));
    } else {
      mxSetField(plhs[0], i, "swing_foot", mxCreateString("left"));
    }

    Matrix<double, 2, 1> r_cop;
    r_cop = p->cop.translation().head(2);
    mxSetField(plhs[0], i, "r_cop", eigenToMatlab(r_cop));
  }

}