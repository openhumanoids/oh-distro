/* 
 * A c++ version of (significant pieces of) the QPController.m mimoOutput method. 
 *
 * Todo:
 *   handle the no supports case (arguments into mex starting with B_ls will currently fail)
 *   use fixed-size matrices (or at least pre-allocated)
 *       for instance: #define nq 
 *       set MaxRowsAtCompileTime (http://eigen.tuxfamily.org/dox/TutorialMatrixClass.html)
 *   some matrices might be better off using RowMajor
 */



#include <mex.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <set>
#include <vector>
#include <Eigen/Dense>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
//#include <Eigen/Sparse>
#include <gurobi_c++.h>

#include "mexmaps/MapLib.hpp"
#include <maps/ViewBase.hpp>

#include "RigidBodyManipulator.h"

const int m_surface_tangents = 2;  // number of faces in the friction cone approx
const double mu = .5;  // coefficient of friction

using namespace std;

struct QPControllerData {
  GRBenv *env;
  RigidBodyManipulator* r;
  double w; // objective function weight
  double slack_limit; // maximum absolute magnitude of acceleration slack variable values
  int Rnnz,*Rind1,*Rind2; double* Rval; // my sparse representation of R_con - the quadratic cost on input
  MatrixXd B, B_con, B_free;
  set<int> free_dof, con_dof, free_inputs, con_inputs; 
  VectorXd umin_con, umax_con;
  ArrayXd umin,umax;
  void* map_ptr;
};

mxArray* myGetProperty(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetProperty(pobj,0,propname);
  if (!pm) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","can't find object property %s", propname);
  return pm;
}

template <typename DerivedA, typename DerivedB>
void getRows(set<int> &rows, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub)
{
  if (rows.size()==M.rows()) {
    Msub = M; 
    return;
  }
  
  int i=0;
  for (set<int>::iterator iter=rows.begin(); iter!=rows.end(); iter++)
    Msub.row(i++) = M.row(*iter);
}

template <typename DerivedA, typename DerivedB>
void getCols(set<int> &cols, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub)
{
  if (cols.size()==M.cols()) {
    Msub = M;
    return;
  }
  int i=0;
  for (set<int>::iterator iter=cols.begin(); iter!=cols.end(); iter++)
    Msub.col(i++) = M.col(*iter);
}

// helper function for shuffling debugging data back into matlab
template <int Rows, int Cols>
mxArray* eigenToMatlab(Matrix<double,Rows,Cols> &m)
{
  mxArray* pm = mxCreateDoubleMatrix(m.rows(),m.cols(),mxREAL);
  if (m.rows()*m.cols()>0)
    memcpy(mxGetPr(pm),m.data(),sizeof(double)*m.rows()*m.cols());
  return pm;
}

template <typename DerivedA,typename DerivedB>
int myGRBaddconstrs(GRBmodel *model, MatrixBase<DerivedA> const & A, MatrixBase<DerivedB> const & b, char sense, double sparseness_threshold = 1e-10)
{
  int i,j,nnz,error;
/*
  // todo: it seems like I should just be able to do something like this:
  SparseMatrix<double,RowMajor> sparseAeq(Aeq.sparseView());
  sparseAeq.makeCompressed();
  error = GRBaddconstrs(model,nq_con,sparseAeq.nonZeros(),sparseAeq.InnerIndices(),sparseAeq.OuterStarts(),sparseAeq.Values(),beq.data(),NULL);
*/
  
  int *cind = new int[A.cols()];
  double* cval = new double[A.cols()];
  for (i=0; i<A.rows(); i++) {
    nnz=0;
    for (j=0; j<A.cols(); j++) {
      if (abs(A(i,j))>sparseness_threshold) {
        cval[nnz] = A(i,j);
        cind[nnz++] = j;
      }
    }
    error = GRBaddconstr(model,nnz,cind,cval,sense,b(i),NULL);
    if (error) break;
  }
  
  delete[] cind;
  delete[] cval;
  return error;
}


void collisionDetect(void* map_ptr, Vector3d const & contact_pos, Vector3d &pos, Vector3d &normal)
{
  Vector3f floatPos, floatNormal;
  if (map_ptr) {
    auto state = static_cast<mexmaps::MapHandle*>(map_ptr);
    if (state != NULL) {
      auto view = state->getView();
      if (view != NULL) {
        if (view->getClosest(contact_pos.cast<float>(),floatPos,floatNormal)) {
          pos = floatPos.cast<double>();
          normal = floatNormal.cast<double>();
          return;
        }
      }
    }
  } else {
    mexPrintf("Warning: using 0,0,1 as normal");
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
  int i,j;

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
    int nq_con = pdata->con_dof.size();

    pm = myGetProperty(pobj,"free_inputs");
    pr = mxGetPr(pm);
    for (i=0; i<mxGetNumberOfElements(pm); i++)
      pdata->free_inputs.insert((int)pr[i] - 1);

    pm = myGetProperty(pobj,"con_inputs");
    pr = mxGetPr(pm);
    for (i=0; i<mxGetNumberOfElements(pm); i++)
      pdata->con_inputs.insert((int)pr[i] - 1);
    int nu_con = pdata->con_inputs.size();
    
    // get robot mex model ptr
    if (!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2])!=1)
      mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the third argument should be the robot mex ptr");
    memcpy(&(pdata->r),mxGetData(prhs[2]),sizeof(pdata->r));
    
    pdata->B.resize(mxGetM(prhs[3]),mxGetN(prhs[3]));
    memcpy(pdata->B.data(),mxGetPr(prhs[3]),sizeof(double)*mxGetM(prhs[3])*mxGetN(prhs[3]));

    int nq = pdata->r->num_dof, nu = pdata->B.cols();
    
    pdata->umin.resize(nu);
    pdata->umax.resize(nu);
    memcpy(pdata->umin.data(),mxGetPr(prhs[4]),sizeof(double)*nu);
    memcpy(pdata->umax.data(),mxGetPr(prhs[5]),sizeof(double)*nu);
    pdata->umin_con.resize(nu_con);
    pdata->umax_con.resize(nu_con);
    getRows(pdata->con_inputs,pdata->umin.matrix(),pdata->umin_con);
    getRows(pdata->con_inputs,pdata->umax.matrix(),pdata->umax_con);

    {
      pdata->B_con.resize(pdata->con_dof.size(),pdata->con_inputs.size());
      MatrixXd tmp(nq,pdata->con_inputs.size());
      getCols(pdata->con_inputs,pdata->B,tmp);
      getRows(pdata->con_dof,tmp,pdata->B_con);
    
      if (pdata->free_dof.size()>0 && pdata->free_inputs.size()>0) {
        pdata->B_free.resize(pdata->free_dof.size(),pdata->free_inputs.size());
        tmp.resize(nq,pdata->free_inputs.size());
        getCols(pdata->free_inputs,pdata->B,tmp);
        getRows(pdata->free_dof,tmp,pdata->B_free);
      }
    }

    {
      pm = myGetProperty(pobj,"R");
      Map<MatrixXd> R(mxGetPr(pm),mxGetM(pm),mxGetN(pm));
      MatrixXd R_con(nu_con,nu_con), tmp(nu,nu_con);
      getCols(pdata->con_inputs,R,tmp);
      getRows(pdata->con_inputs,tmp,R_con);
    
      int nnz=0;
      for (i=0; i<nu_con; i++)
        for (j=0; j<nu_con; j++)
          if (abs(R_con(i,j))>1e-10) nnz++;
      pdata->Rind1 = new int[nnz];
      pdata->Rind2 = new int[nnz];
      pdata->Rval = new double[nnz];
      pdata->Rnnz = nnz;
      nnz=0;
      for (i=0; i<nu_con; i++)
        for (j=0; j<nu_con; j++)
          if (abs(R_con(i,j))>1e-10) {
            pdata->Rind1[nnz] = i+nq_con;
            pdata->Rind2[nnz] = j+nq_con;
            pdata->Rval[nnz++] = R_con(i,j);
          }
    }    

     // get the map ptr back from matlab
     if (!mxIsNumeric(prhs[6]) || mxGetNumberOfElements(prhs[6])!=1)
     mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the seventh argument should be the map ptr");
     memcpy(&pdata->map_ptr,mxGetPr(prhs[6]),sizeof(pdata->map_ptr));
    
//    pdata->map_ptr = NULL;
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
      nq_free = pdata->free_dof.size(),
      nq_con = pdata->con_dof.size(),
      nu_con = pdata->con_inputs.size();
  const int dim = 3, // 3D
      nd = 2*m_surface_tangents; // for friction cone approx, hard coded for now
  
  assert(nq_con+nq_free == nq);
  assert(nu_con+pdata->free_inputs.size() == nu);

  int narg=1;

  Map< VectorXd > q_ddot_des(mxGetPr(prhs[narg++]),nq);
  
  double *q = mxGetPr(prhs[narg++]);
  double *qd = &q[nq];
  
  set<int> active_supports;
  pr = mxGetPr(prhs[narg]);
  for (i=0; i<mxGetNumberOfElements(prhs[narg]); i++)
    active_supports.insert((int)pr[i] - 1);
  narg++;
  
  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==2);
  Map< Matrix<double,4,2> > B_ls(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==2);
  Map< Matrix2d > Qy(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==2);
  Map< Matrix2d > R_ls(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==4);
  Map< Matrix<double,2,4> > C_ls(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==2);
  Map< Matrix2d > D_ls(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==4);
  Map< Matrix4d > S(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==1);
  Map< Vector4d > s1(mxGetPr(prhs[narg++]));
  
  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==1);
  Map< Vector4d > x0(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==1);
  Map< Vector2d > u0(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==1);
  Map< Vector2d > y0(mxGetPr(prhs[narg++]));

  Matrix2d R_DQyD_ls = R_ls + D_ls.transpose()*Qy*D_ls;
  
  pdata->r->doKinematics(q,false,qd);
  
  MatrixXd H(nq,nq);
  VectorXd C(nq);
  
  pdata->r->HandC(q,qd,(MatrixXd*)NULL,H,C,(MatrixXd*)NULL,(MatrixXd*)NULL);

  MatrixXd H_con(nq_con,nq), H_free(nq_free,nq);
  VectorXd C_con(nq_con), C_free(nq_free);
  
  getRows(pdata->con_dof,H,H_con);
  getRows(pdata->con_dof,C,C_con);
  
  if (nq_free>0) {
    getRows(pdata->free_dof,H,H_free);
    getRows(pdata->free_dof,C,C_free);
  }
  
  Vector3d xcom;
  MatrixXd J(3,nq), Jdot(3,nq);
  // consider making all J's into row-major
  
  pdata->r->getCOM(xcom);
  pdata->r->getCOMJac(J);
  pdata->r->getCOMJacDot(Jdot);

  Map<VectorXd> qdvec(qd,nq);
  VectorXd qd_con(nq_con);
  getRows(pdata->con_dof,qdvec,qd_con);
  
//  VectorXd phi;
  MatrixXd Jz,Jp,Jpdot,D;
  int nc = contactConstraints(pdata,active_supports,Jz,D,Jp,Jpdot);
  int neps = nc*dim;

  Vector4d x_bar,xlimp;
  MatrixXd Jz_con(Jz.rows(),nq_con),Jp_con(Jp.rows(),nq_con),Jpdot_con(Jpdot.rows(),nq_con),D_con(nq_con,D.cols());
  if (nc>0) {
    xlimp << xcom.topRows(2),J.topRows(2)*qdvec;
    x_bar << xlimp.topRows(2)-x0.topRows(2),xlimp.bottomRows(2)-x0.bottomRows(2);
    getCols(pdata->con_dof,Jz,Jz_con);
    getRows(pdata->con_dof,D,D_con);
    getCols(pdata->con_dof,Jp,Jp_con);
    getCols(pdata->con_dof,Jpdot,Jpdot_con);
  }
  

  //---------------------------------------------------------------------
  // Free DOF cost function ----------------------------------------------
  
  VectorXd qdd_free(nq_free);
  if (nq_free > 0) {
    MatrixXd Hqp(nq_free,nq_free);
    RowVectorXd fqp(nq_free);

    if (nc > 0) {
      VectorXd q_ddot_des_free(nq_free);
      getRows(pdata->free_dof,q_ddot_des,q_ddot_des_free);
      
      MatrixXd J_free(2,nq_free),Jdot_free(2,nq_free);
      VectorXd qd_free(nq_free);
      getCols(pdata->free_dof,J.topRows(2),J_free);
      getCols(pdata->free_dof,Jdot.topRows(2),Jdot_free);
      getRows(pdata->free_dof,qdvec,qd_free);
      
      // approximate quadratic cost for free dofs with the appropriate matrix block
      Hqp = J_free.transpose()*R_DQyD_ls*J_free;
      Hqp += pdata->w*MatrixXd::Identity(nq_free,nq_free);
              
      fqp = (C_ls*xlimp).transpose()*Qy*D_ls*J_free;
      fqp += (Jdot_free*qd_free).transpose()*R_DQyD_ls*J_free;
      fqp += (S*x_bar + 0.5*s1).transpose()*B_ls*J_free;
      fqp -= u0.transpose()*R_DQyD_ls*J_free;
      fqp -= y0.transpose()*Qy*D_ls*J_free;
      fqp -= pdata->w*q_ddot_des_free.transpose();
      
      // solve for qdd_free unconstrained
      qdd_free = -Hqp.ldlt().solve(fqp.transpose());
    } else {
      // qdd_free = q_ddot_des_free;
      getRows(pdata->free_dof,q_ddot_des,qdd_free);
    }        
    
  }

  int nf = nc+nc*nd; // number of contact force variables
  int nparams = nq_con+nu_con+nf+neps;
  
  GRBmodel *model = NULL;

  // set obj,lb,up
  VectorXd lb(nparams), ub(nparams);
  lb.topRows(nq_con) = -1e3*VectorXd::Ones(nq_con);
  ub.topRows(nq_con) = 1e3*VectorXd::Ones(nq_con);
  lb.block(nq_con,0,nu_con,1) = pdata->umin_con;
  ub.block(nq_con,0,nu_con,1) = pdata->umax_con;
  lb.block(nq_con+nu_con,0,nf,1) = VectorXd::Zero(nf);
  ub.block(nq_con+nu_con,0,nf,1) = 500*VectorXd::Ones(nf);
  lb.bottomRows(neps) = -pdata->slack_limit*VectorXd::Ones(neps);
  ub.bottomRows(neps) = pdata->slack_limit*VectorXd::Ones(neps);
  
  error = GRBnewmodel(pdata->env,&model,"QPController",nparams,NULL,lb.data(),ub.data(),NULL,NULL);
  
  //----------------------------------------------------------------------
  // QP cost function ----------------------------------------------------
  //
  //  min: quad(Jdot*qd + J*qdd,R_ls)+quad(C*x+D*(Jdot*qd + J*qdd),Qy) + (2*x'*S + s1')*(A*x + B*(Jdot*qd + J*qdd)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)

  {      
    VectorXd q_ddot_des_con(nq_con);
    getRows(pdata->con_dof,q_ddot_des,q_ddot_des_con);
    if (nc > 0) {
      MatrixXd Hqp_con(nq_con,nq_con);
      RowVectorXd fqp_con(nq_con);
      
      MatrixXd J_con(2,nq_con),Jdot_con(2,nq_con);
      getCols(pdata->con_dof,J.topRows(2),J_con);
      getCols(pdata->con_dof,Jdot.topRows(2),Jdot_con);
      
      Hqp_con = J_con.transpose()*R_DQyD_ls*J_con;
      Hqp_con += pdata->w*MatrixXd::Identity(nq_con,nq_con);
              
      fqp_con = (C_ls*xlimp).transpose()*Qy*D_ls*J_con;
      fqp_con += (Jdot_con*qd_con).transpose()*R_DQyD_ls*J_con;
      fqp_con += (S*x_bar + 0.5*s1).transpose()*B_ls*J_con;
      fqp_con -= u0.transpose()*R_DQyD_ls*J_con;
      fqp_con -= y0.transpose()*Qy*D_ls*J_con;
      fqp_con -= pdata->w*q_ddot_des_con.transpose();

      // Q(1:nq_con,1:nq_con) = Hqp_con
      for (i=0; i<nq_con; i++)
        for (j=0; j<nq_con; j++)
          if (abs(Hqp_con(i,j))>1e-10) 
            error = GRBaddqpterms(model,1,&i,&j,&(Hqp_con(i,j)));

      // obj(1:nq_con) = 2*fqp_con
      fqp_con *= 2;
      error = GRBsetdblattrarray(model,"Obj",0,nq_con,fqp_con.data());
      
      // quadratic slack var cost, Q(nparams-neps:end,nparams-neps:end)=eye(neps)
      double cost = .001;
      for (i=nparams-neps; i<nparams; i++) {
        error = GRBaddqpterms(model,1,&i,&i,&cost);
      }
    } else {
      // Q(1:nq_con,1:nq_con) = eye(nq_con)
      double cost = 1;
      for (i=0; i<nq_con; i++) {  
        error = GRBaddqpterms(model,1,&i,&i,&cost);
      }
      // obj(1:nq_con) = -qddot_des_con
      q_ddot_des_con *= -1;
      error = GRBsetdblattrarray(model,"Obj",0,nq_con,q_ddot_des_con.data());
    } 
    
    // quadratic input cost Q(nq_con+(1:nu_con),nq_con+(1:nu_con))=R
    error = GRBaddqpterms(model,pdata->Rnnz,pdata->Rind1,pdata->Rind2,pdata->Rval);
  }      
  
  { // constrained dynamics
    //  H*qdd - B*u - J*lambda - Dbar*beta = -C
    MatrixXd H_con_con(nq_con,nq_con), H_con_free(nq_con,nq_free);
    getCols(pdata->con_dof,H_con,H_con_con);
    
    MatrixXd Aeq;
    VectorXd beq(nq_con);
    if (nc>0) {
      Aeq.resize(nq_con,nparams-neps);
      Aeq.block(0,nq_con+nu_con,nq_con,nc) = -Jz_con.transpose();
      Aeq.block(0,nq_con+nu_con+nc,nq_con,nc*nd) = -D_con;
    } else {
      Aeq.resize(nq_con,nq_con+nu_con);
    }
    Aeq.topLeftCorner(nq_con,nq_con) = H_con_con;
    Aeq.block(0,nq_con,nq_con,nu_con) = -pdata->B_con;
    beq = -C_con;
    if (nq_free>0) {
      getCols(pdata->free_dof,H_con,H_con_free);
      beq -= H_con_free*qdd_free;
    }      

    error = myGRBaddconstrs(model,Aeq,beq,GRB_EQUAL);
  }

  if (nc > 0) {
    // relative acceleration constraint
    MatrixXd Aeq(neps,nparams);
    VectorXd beq(neps);
    
    Aeq.topLeftCorner(neps,nq_con) = Jp_con;
    Aeq.block(0,nq_con,neps,nu_con+nf) = MatrixXd::Zero(neps,nu_con+nf);  // todo:  this is very inefficient
    Aeq.topRightCorner(neps,neps) = MatrixXd::Identity(neps,neps);
    beq = (-Jpdot_con - 1.0*Jp_con)*qd_con;
    
    error = myGRBaddconstrs(model,Aeq,beq,GRB_EQUAL);
  }    
  
  if (nc>0) {
    // linear friction constraints
    int cind[1+nd];
    double cval[1+nd] = {-mu,1,1,1,1}; // this will have to change if m_surface_tangents changes!
    
    for (i=0; i<nc; i++) {
      // -mu*lambda[i] + sum(beta[i]s) <= 0
      cind[0] = nq_con+nu_con+i;
      for (j=0; j<nd; j++) cind[j+1]=nq_con+nu_con+nc+i*nd+j;
      error = GRBaddconstr(model,1+nd,cind,cval,GRB_LESS_EQUAL,0,NULL);
    }
  }    
  
  error = GRBupdatemodel(model);
  error = GRBoptimize(model);
  
  VectorXd alpha(nparams);
  error = GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, nparams, alpha.data());
  
  //----------------------------------------------------------------------
  // Solve for free inputs -----------------------------------------------
  VectorXd y(nu);
  if (nq_free > 0) {
    VectorXd qdd(nq);
    set<int>::iterator iter;
    i=0;
    for (iter=pdata->free_dof.begin(); iter!=pdata->free_dof.end(); iter++)
      qdd(*iter) = qdd_free(i++);
    i=0;
    for (iter=pdata->con_dof.begin(); iter!=pdata->con_dof.end(); iter++)
      qdd(*iter) = alpha(i++);
  
    VectorXd u_free = pdata->B_free.jacobiSvd(ComputeThinU|ComputeThinV).solve(H_free*qdd + C_free);

    i=0;
    for (iter=pdata->free_inputs.begin(); iter!=pdata->free_inputs.end(); iter++)
      y(*iter) = u_free(i++);
    i=0;
    for (iter=pdata->con_inputs.begin(); iter!=pdata->con_inputs.end(); iter++) {
      y(*iter) = alpha(nq_con + i);
      i++;
    }    
    
    // saturate inputs
    ArrayXd tmp = pdata->umin.max(y.array());
    y = tmp.min(pdata->umax).matrix();
  } else {
    y = alpha.block(nq,0,nu,1); 
  }
  
  if (nlhs>0) plhs[0] = eigenToMatlab(y);
    
  if (nlhs>1) {  // return model.Q (for unit testing)
    int qnz;
    error = GRBgetintattr(model,"NumQNZs",&qnz);
    int *qrow = new int[qnz], *qcol = new int[qnz];
    double* qval = new double[qnz];
    error = GRBgetq(model,&qnz,qrow,qcol,qval);
    plhs[1] = mxCreateDoubleMatrix(nparams,nparams,mxREAL);
    double* pm = mxGetPr(plhs[1]);
    memset(pm,0,sizeof(double)*nparams*nparams);
    for (i=0; i<qnz; i++)
      pm[qrow[i]+nparams*qcol[i]] = qval[i];
    delete[] qrow;
    delete[] qcol;
    delete[] qval;
    
    if (nlhs>2) {  // return model.obj (for unit testing)
      plhs[2] = mxCreateDoubleMatrix(1,nparams,mxREAL);
      error = GRBgetdblattrarray(model, "Obj", 0, nparams, mxGetPr(plhs[2]));

      if (nlhs>3) {  // return model.A (for unit testing)
        int numcon;
        error = GRBgetintattr(model,"NumConstrs",&numcon);
        plhs[3] = mxCreateDoubleMatrix(numcon,nparams,mxREAL);
        double *pm = mxGetPr(plhs[3]);
        for (i=0; i<numcon; i++)
          for (j=0; j<nparams; j++)
            error = GRBgetcoeff(model,i,j,&pm[i+j*numcon]);
        
        if (nlhs>4) {  // return model.rhs (for unit testing)
          plhs[4] = mxCreateDoubleMatrix(numcon,1,mxREAL);
          GRBgetdblattrarray(model,"RHS",0,numcon,mxGetPr(plhs[4]));
        } 
        
        if (nlhs>5) { // return model.sense
          char* sense = new char[numcon+1];
          GRBgetcharattrarray(model,"Sense",0,numcon,sense);
          sense[numcon]='\0';
          plhs[5] = mxCreateString(sense);
          // delete[] sense;  // it seems that I'm not supposed to free this
        }
        
        if (nlhs>6) plhs[6] = eigenToMatlab(lb);
        if (nlhs>7) plhs[7] = eigenToMatlab(ub);
      }
    }
  }
  
  GRBfreemodel(model);
//  GRBfreeenv(env);
} 
