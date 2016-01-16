#include "drake/util/drakeMexUtil.h"
#include "drake/systems/controllers/controlUtil.h"

using namespace Eigen;

struct FootstepShiftData {
  RigidBodyTree* r;
  double t_prev;
  double sample_dt;
  int lfoot_body_index;
  int rfoot_body_index;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: ptr = footstepPlanShiftmex(0,robot_obj,sample_rate); y=footstepPlanShiftmex(ptr,t,x,fc_left,fc_right,lfoot_des,rfoot_des,plan_shift)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");
  
  struct FootstepShiftData* pdata;

  if (mxGetScalar(prhs[0])==0) { // then construct the data object and return
    pdata = new struct FootstepShiftData;
    
    // get robot mex model ptr
    if (!mxIsNumeric(prhs[1]) || mxGetNumberOfElements(prhs[1])!=1)
      mexErrMsgIdAndTxt("DRC:footstepPlanShiftmex:BadInputs","the second argument should be the robot mex ptr");
    memcpy(&(pdata->r),mxGetData(prhs[1]),sizeof(pdata->r));
        
    double sample_rate = mxGetScalar(prhs[2]);
    pdata->sample_dt = 1.0/sample_rate; 

    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:footstepPlanShiftmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
     
    pdata->t_prev = -1;
    pdata->lfoot_body_index = pdata->r->findLinkId("l_foot", 0);
    pdata->rfoot_body_index = pdata->r->findLinkId("r_foot", 0);

    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));
     
    return;
  }
  
  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:footstepPlanShiftmex:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

  int nq = pdata->r->num_positions;
  int narg = 1;

  double t = mxGetScalar(prhs[narg++]);

  double *q_ptr = mxGetPr(prhs[narg++]);
  double *qd_ptr = &q_ptr[nq];
  Map<VectorXd> q(q_ptr, nq);
  Map<VectorXd> qd(qd_ptr, pdata->r->num_velocities);
  
  int fc_left = (int) mxGetScalar(prhs[narg++]);
  int fc_right = (int) mxGetScalar(prhs[narg++]);

  assert(mxGetM(prhs[narg])==3); assert(mxGetN(prhs[narg])==1);
  Map< Vector3d > lfoot_des(mxGetPr(prhs[narg++]));
  assert(mxGetM(prhs[narg])==3); assert(mxGetN(prhs[narg])==1);
  Map< Vector3d > rfoot_des(mxGetPr(prhs[narg++]));
  assert(mxGetM(prhs[narg])==3); assert(mxGetN(prhs[narg])==1);
  Map< Vector3d > plan_shift(mxGetPr(prhs[narg++]));
  
  if (t-pdata->t_prev >= pdata->sample_dt) {
    KinematicsCache<double> cache = pdata->r->doKinematics(q,qd,false);
    if (fc_left == 1) {
      Vector3d lfoot_pos;

      lfoot_pos = pdata->r->transformPoints(cache, Vector3d::Zero().eval(), pdata->lfoot_body_index, 0);
      plan_shift = lfoot_des - lfoot_pos;
    }
    else if (fc_right == 1) {
      Vector3d rfoot_pos;
      rfoot_pos = pdata->r->transformPoints(cache, Vector3d::Zero().eval(), pdata->rfoot_body_index, 0);
      plan_shift = rfoot_des - rfoot_pos;
    }
  }
  Vector3d ret = plan_shift;
  plhs[0] = eigenToMatlab(ret);
}
