/* 
 * A c++ version of (significant pieces of) the QPController.m mimoOutput method. 
 *
 */

#include <mex.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <set>
#include <vector>
#include <Eigen/Dense>
#include <gurobi_c++.h>

#include "RigidBodyManipulator.h"

const int m_surface_tangents = 2;

using namespace std;

struct QPControllerData {
  GRBenv *env;
  RigidBodyManipulator* r;
  double w; // objective function weight
  double slack_limit; // maximum absolute magnitude of acceleration slack variable values
  MatrixXd R; // quadratic input cost matrix
  MatrixXd B, B_con, B_free;
  void* map_ptr;

  set<int> free_dof, con_dof, free_inputs, con_inputs; 
  MatrixXd q_selector_con_left, q_selector_con_right, 
          u_selector_con_left, u_selector_con_right,
          q_selector_free_left, q_selector_free_right,
          u_selector_free_left, u_selector_free_right;
};

mxArray* myGetProperty(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetProperty(pobj,0,propname);
  if (!pm) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","can't find object property %s", propname);
  return pm;
}


// helper function for shuffling debugging data back into matlab
template <int DerivedA, int DerivedB>
mxArray* eigenToMatlab(Matrix<double,DerivedA,DerivedB> &m)
{
  mxArray* pm = mxCreateDoubleMatrix(m.rows(),m.cols(),mxREAL);
  if (m.rows()*m.cols()>0)
    memcpy(mxGetPr(pm),m.data(),sizeof(double)*m.rows()*m.cols());
  return pm;
}

void collisionDetect(void* map_ptr, Vector3d const & contact_pos, Vector3d &pos, Vector3d &normal)
{
  if (map_ptr) {
    mexErrMsgTxt("Will have to connect to map api soon, but haven't done it yet");
  } else {
    pos << contact_pos.topRows(2), 0;
    normal << 0,0,1;
  }
  // just assume mu = 1 for now
}

void surfaceTangents(const Vector3d & normal, Matrix<double,3,m_surface_tangents> & d)
{
  Vector3d t1,t2;
  double theta;
  
  if (1 - normal(2) < 10e-8) { // handle the unit-normal case (since it's unit length, just check z)
    t1 << 1,0,0;
  } else { // now the general case
    t1 << normal(2), -normal(1), 0; // normal.cross([0;0;1])
    t1 /= sqrt(normal(1)*normal(1) + normal(2)*normal(2));
  }
      
  t2 = t1.cross(normal);
      
  for (int k=0; k<m_surface_tangents; k++) {
    theta = k*M_PI/m_surface_tangents;
    d.col(k)=cos(theta)*t1 + sin(theta)*t2;
  }
}

int contactConstraints(struct QPControllerData* pdata, set<int> body_idx, MatrixXd &n, MatrixXd &D, MatrixXd &Jp, MatrixXd &Jpdot) 
{
  int i, j, k=0, nc = pdata->r->getNumContacts(body_idx), nq = pdata->r->num_dof;

//  phi.resize(nc);
  n.resize(nc,nq);
  D.resize(nq,nc*2*m_surface_tangents);
  Jp.resize(3*nc,nq);
  Jpdot.resize(3*nc,nq);
  
  Vector3d contact_pos,pos,normal; Vector4d tmp;
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  
  for (set<int>::iterator iter = body_idx.begin(); iter!=body_idx.end(); iter++) {
    RigidBody* b = &(pdata->r->bodies[*iter]);
    nc = b->contact_pts.cols();
    if (nc>0) {
      for (i=0; i<nc; i++) {
        tmp = b->contact_pts.col(i);
        pdata->r->forwardKin(*iter,tmp,0,contact_pos);
        pdata->r->forwardJac(*iter,tmp,0,J);

        collisionDetect(pdata->map_ptr,contact_pos,pos,normal);
        
// phi is not even being used right now        
//        pos -= contact_pos;  // now -rel_pos in matlab version
//        phi(k) = pos.norm();
//        if (pos.dot(normal)>0) phi(k)=-phi(k);

        surfaceTangents(normal,d);

        n.row(k) = normal.transpose()*J;
        for (j=0; j<m_surface_tangents; j++) {
          D.col(2*k*m_surface_tangents+j) = J.transpose()*d.col(j);
          D.col((2*k+1)*m_surface_tangents+j) = -D.col(2*k*m_surface_tangents+j);
        }

        // store away kin sols into Jp and Jpdot
        // NOTE: I'm cheating and using a slightly different ordering of J and Jdot here
        Jp.block(3*k,0,3,nq) = J;
        pdata->r->forwardJacDot(*iter,tmp,J);
        Jpdot.block(3*k,0,3,nq) = J;
        
        k++;
      }
    }
  }
  
  return k;
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  int error;
  if (nrhs<1) mexErrMsgTxt("usage: ptr = QPControllermex(0,control_obj,robot_obj); alpha=QPControllermex(ptr,...,...)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");
  
  struct QPControllerData* pdata;
  mxArray* pm;
  double* pr;
  int i;

  if (mxGetScalar(prhs[0])==0) { // then construct the data object and return
    pdata = new struct QPControllerData;
    
    // get control object properties
    const mxArray* pobj = prhs[1];
    
    pm= myGetProperty(pobj,"w");
    pdata->w = mxGetScalar(pm);
    
    pm = myGetProperty(pobj,"slack_limit");
    pdata->slack_limit = mxGetScalar(pm);

    pm = myGetProperty(pobj,"free_dof");
    pr = mxGetPr(pm);
    for (i=0; i<mxGetNumberOfElements(pm); i++)
      pdata->free_dof.insert((int)pr[i] - 1);
    
    pm = myGetProperty(pobj,"con_dof");
    pr = mxGetPr(pm);
    for (i=0; i<mxGetNumberOfElements(pm); i++)
      pdata->con_dof.insert((int)pr[i] - 1);

    pm = myGetProperty(pobj,"free_inputs");
    pr = mxGetPr(pm);
    for (i=0; i<mxGetNumberOfElements(pm); i++)
      pdata->free_inputs.insert((int)pr[i] - 1);

    pm = myGetProperty(pobj,"con_inputs");
    pr = mxGetPr(pm);
    for (i=0; i<mxGetNumberOfElements(pm); i++)
      pdata->con_inputs.insert((int)pr[i] - 1);
    
    // get robot mex model ptr
    if (!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2])!=1)
      mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the third argument should be the robot mex ptr");
    memcpy(&(pdata->r),mxGetData(prhs[2]),sizeof(pdata->r));
    
    pdata->B.resize(mxGetM(prhs[3]),mxGetN(prhs[3]));
    memcpy(pdata->B.data(),mxGetPr(prhs[3]),sizeof(double)*mxGetM(prhs[3])*mxGetN(prhs[3]));

    int nq = pdata->r->num_dof, nu = pdata->B.cols();
    
    // pre-allocate a bunch of these to keep things fast inside the loop
    // q_selector_con_left*A is equivalent to A(obj.con_dof,:)
    // A*q_selector_con_right is equivalent to A(:,obj.con_dof)
    // etc, etc...
    set<int>::iterator iter;
    int k;
    pdata->q_selector_con_left = MatrixXd::Zero(pdata->con_dof.size(),nq);
    pdata->u_selector_con_left = MatrixXd::Zero(pdata->con_inputs.size(),nu);
    k=0; for (iter=pdata->con_dof.begin(); iter!=pdata->con_dof.end(); iter++) pdata->q_selector_con_left(k++,*iter)=1;
    k=0; for (iter=pdata->con_inputs.begin(); iter!=pdata->con_inputs.end(); iter++) pdata->u_selector_con_left(k++,*iter)=1;
    pdata->q_selector_con_right = pdata->q_selector_con_left.transpose();
    pdata->u_selector_con_right = pdata->u_selector_con_left.transpose();
  
    if (pdata->free_dof.size()>0) {
      pdata->q_selector_free_left = MatrixXd::Zero(pdata->free_dof.size(),nq);
      k=0; for (iter=pdata->free_dof.begin(); iter!=pdata->free_dof.end(); iter++) pdata->q_selector_free_left(k++,*iter)=1;
      pdata->q_selector_free_right = pdata->q_selector_free_left.transpose();
    }
    if (pdata->free_inputs.size()>0) {
      pdata->u_selector_free_left = MatrixXd::Zero(pdata->free_inputs.size(),nu);
      k=0; for (iter=pdata->free_inputs.begin(); iter!=pdata->free_inputs.end(); iter++) pdata->u_selector_free_left(k++,*iter)=1;
      pdata->u_selector_free_right = pdata->u_selector_free_left.transpose();
    }
    
    pdata->B_con = pdata->q_selector_con_left*pdata->B*pdata->u_selector_con_right;
    if (pdata->free_dof.size()>0 && pdata->free_inputs.size()>0)
      pdata->B_free = pdata->q_selector_free_left*pdata->B*pdata->u_selector_free_right;

    pm = myGetProperty(pobj,"R");
    pdata->R.resize(mxGetM(pm),mxGetN(pm));
    memcpy(pdata->R.data(),mxGetPr(pm),sizeof(double)*mxGetM(pm)*mxGetN(pm));
    
    pdata->map_ptr = NULL;
    if (!pdata->map_ptr)
      mexWarnMsgTxt("Map ptr is NULL.  Assuming flat terrain at z=0");
    
    // create gurobi environment
    error = GRBloadenv(&(pdata->env),NULL);

    // set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
    error = GRBsetintparam(pdata->env,"outputflag",0);
    error = GRBsetintparam(pdata->env,"method",2);
    error = GRBsetintparam(pdata->env,"presolve",0);
    error = GRBsetintparam(pdata->env,"bariterlimit",20);
    error = GRBsetintparam(pdata->env,"barhomogenous",0);
    error = GRBsetdblparam(pdata->env,"barconvtol",0.0005);
  
    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:constructModelmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));
    return;
  }
  
  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

  int nu = pdata->B.cols(), 
      nq = pdata->r->num_dof,
      dim = 3, // 3D
      nd = 4, // for friction cone approx, hard coded for now
      nq_free = pdata->free_dof.size(),
      nq_con = pdata->con_dof.size(),
      nu_con = pdata->con_inputs.size();
  
  double *q = mxGetPr(prhs[1]);
  double *qd = &q[nq];
  
  set<int> active_supports;
  pr = mxGetPr(prhs[2]);
  for (i=0; i<mxGetNumberOfElements(prhs[2]); i++)
    active_supports.insert((int)pr[i] - 1);
  
  pdata->r->doKinematics(q,false,qd);
  
  MatrixXd H(nq,nq);
  VectorXd C(nq);
  
  pdata->r->HandC(q,qd,(MatrixXd*)NULL,H,C,(MatrixXd*)NULL,(MatrixXd*)NULL);

  MatrixXd H_con = pdata->q_selector_con_left*H,
          C_con = pdata->q_selector_con_left*C,
          H_free,C_free;
  
  if (nq_free>0) {
    H_free = pdata->q_selector_free_left*H;
    C_free = pdata->q_selector_free_left*C;
  }
  
  // todo: preallocate everything like this!
  Vector3d xcom;
  MatrixXd J(3,nq), Jdot(3,nq);
  
  pdata->r->getCOM(xcom);
  pdata->r->getCOMJac(J);
  pdata->r->getCOMJacDot(Jdot);

  // todo:  consider avoiding this copy 
//  J = J.topRows(2);
//  Jdot = Jdot.topRows(2);
  
//  VectorXd phi;
  MatrixXd Jz,Jp,Jpdot,D;
  int nc = contactConstraints(pdata,active_supports,Jz,D,Jp,Jpdot);

  if (nc>0) {
    Jz *= pdata->q_selector_con_right;
    D = pdata->q_selector_con_left*D;
    Jp *= pdata->q_selector_con_right;
    Jpdot *= pdata->q_selector_con_right;
  }

  if (nlhs<13) mexErrMsgTxt("take all my outputs!"); 

  plhs[0] = eigenToMatlab(H_con);
  plhs[1] = eigenToMatlab(C_con);
  plhs[2] = eigenToMatlab(pdata->B_con);
  plhs[3] = eigenToMatlab(H_free);
  plhs[4] = eigenToMatlab(C_free);
  plhs[5] = eigenToMatlab(pdata->B_free);
  plhs[6] = eigenToMatlab(xcom);
  plhs[7] = eigenToMatlab(J);
  plhs[8] = eigenToMatlab(Jdot);
  plhs[9] = eigenToMatlab(Jz);
  plhs[10] = eigenToMatlab(D);
  plhs[11] = eigenToMatlab(Jp);
  plhs[12] = eigenToMatlab(Jpdot);
  
  /*
  { 
  int nx = mxGetN(A), i;
  GRBmodel *model = NULL;

  // set obj,lb,up
  error = GRBnewmodel(pdata->env,&model,"QPController",nx,mxGetPr(obj),mxGetPr(lb),mxGetPr(ub),NULL,NULL);
    
  { // add quadratic cost terms
    // have to convert qcol from the matlab sparse matrix format
    mwIndex *ir = mxGetIr(Q), *jc = mxGetJc(Q), ncol = mxGetN(Q);
    int nnz = jc[ncol]; // the actual number of nonzero entries
    
    if (sizeof(int)==sizeof(mwIndex)) {  // avoid ugly copy if possible
      int *qcol = new int[nnz], k=0;
      for (int col=0; col<ncol; col++)
        while (k<jc[col+1]) {
        qcol[k++] = col;
        }
      error = GRBaddqpterms(model,nnz,reinterpret_cast<int*>(ir),qcol,mxGetPr(Q));    
    } else {
      int *qrow = new int[nnz], *qcol = new int[nnz], k=0;
      for (int col=0; col<ncol; col++)
        while (k<jc[col+1]) {
        qrow[k]   = (int) ir[k];
        qcol[k++] = col;
        }
      error = GRBaddqpterms(model,nnz,qrow,qcol,mxGetPr(Q));    
    }
  }  
  
  mxArray* At;  // A transpose
  { // Ax = rhs constraints
    // have to convert to CSR from matlab sparse matrix format (note, slightly different than qpterms input)
    mexCallMATLAB(1,&At,1,const_cast<mxArray**>(&A),"ctranspose");

    mwIndex *ir = mxGetIr(At), *jc = mxGetJc(At), ncol = mxGetN(At);
    int nc= mxGetM(A), // number of constraints
      nnz = jc[ncol]; // the actual number of nonzero entries

    int *cbeg, *cval;
    char *s = new char[nc]; 
    mxGetString(sense,s,nc);

    if (sizeof(int)==sizeof(mwIndex)) {     // avoid ugly copies if possible
      cbeg=reinterpret_cast<int*>(jc);
      cval=reinterpret_cast<int*>(ir);
    } else {
      cbeg = new int[nc]; cval = new int[nnz];
      for (i=0; i<nc; i++) cbeg[i] = (int) jc[i];
      for (i=0; i<nnz; i++) cval[i] = (int) ir[i];
    }

    error = GRBaddconstrs(model,nc,nnz,cbeg,cval,mxGetPr(At),s,mxGetPr(rhs),NULL);	
  }
  
  error = GRBupdatemodel(model);
  error = GRBoptimize(model);

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(nx,1,mxREAL);
    error = GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, nx, mxGetPr(plhs[0]));
  }
  
  GRBfreemodel(model);
//  GRBfreeenv(env);
  mxDestroyArray(At);
   */
} 
