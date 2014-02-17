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
//#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
//#include <Eigen/Sparse>

#ifdef USE_MAPS

#include "mexmaps/MapLib.hpp"
#include <maps/ViewBase.hpp>

#endif

//#define TEST_FAST_QP

#include "drake/fastQP.h"
#include "drake/gurobiQP.h"
#include "drake/RigidBodyManipulator.h"

const int m_surface_tangents = 2;  // number of faces in the friction cone approx
const double REG = 1e-8;

using namespace std;

struct QPControllerData {
  GRBenv *env;
  RigidBodyManipulator* r;
  void* multi_robot; // optional multi rigid body system
  double w; // objective function weight
  double slack_limit; // maximum absolute magnitude of acceleration slack variable values
  VectorXd umin,umax;
  void* map_ptr;
  
  set<int> active;

  // preallocate memory
  MatrixXd H, H_float, H_act;
  VectorXd C, C_float, C_act;
  MatrixXd B, B_act;
  MatrixXd J, Jdot;
  MatrixXd J_xy, Jdot_xy;
  MatrixXd Hqp;
  RowVectorXd fqp;
  MatrixXd Ag, Agdot; // centroidal momentum matrix
//  MatrixXd Ain_lb_ub, bin_lb_ub;
};

// helper function for shuffling debugging data back into matlab
template <int Rows, int Cols>
mxArray* eigenToMatlab(Matrix<double,Rows,Cols> &m)
{
  mxArray* pm = mxCreateDoubleMatrix(m.rows(),m.cols(),mxREAL);
  if (m.rows()*m.cols()>0)
  	memcpy(mxGetPr(pm),m.data(),sizeof(double)*m.rows()*m.cols());
  return pm;
}

mxArray* myGetProperty(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetProperty(pobj,0,propname);
  if (!pm) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","QPControllermex is trying to load object property '%s', but failed.", propname);
  return pm;
}

mxArray* myGetField(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetField(pobj,0,propname);
  if (!pm) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","QPControllermex is trying to load object field '%s', but failed.", propname);
  return pm;
}

typedef struct _support_state_element
{
  int body_idx;
  set<int> contact_pt_inds;
  int contact_surface;
} SupportStateElement;


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

void collisionDetect(void* map_ptr, Vector3d const & contact_pos, Vector3d &pos, Vector3d *normal, double terrain_height)
{
  if (map_ptr) {
#ifdef USE_MAPS    
    Vector3f floatPos, floatNormal;
    auto state = static_cast<mexmaps::MapHandle*>(map_ptr);
    if (state != NULL) {
      auto view = state->getView();
      if (view != NULL) {
        if (view->getClosest(contact_pos.cast<float>(),floatPos,floatNormal)) {
          pos = floatPos.cast<double>();
          if (normal) *normal = floatNormal.cast<double>();
          return;
        }
      }
    }
#endif      
  } else {
//    mexPrintf("Warning: using 0,0,1 as normal\n");
    pos << contact_pos.topRows(2), terrain_height;
    if (normal) *normal << 0,0,1;
  }
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

int contactPhi(struct QPControllerData* pdata, SupportStateElement& supp, VectorXd &phi, double terrain_height)
{
  RigidBody* b = &(pdata->r->bodies[supp.body_idx]);
	int nc = supp.contact_pt_inds.size();
	phi.resize(nc);

	if (nc<1) return nc;

  Vector3d contact_pos,pos,posB,normal; Vector4d tmp;

  int i=0;
  for (set<int>::iterator pt_iter=supp.contact_pt_inds.begin(); pt_iter!=supp.contact_pt_inds.end(); pt_iter++) {
  	if (*pt_iter<0 || *pt_iter>=b->contact_pts.cols()) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","requesting contact pt %d but body only has %d pts",*pt_iter,b->contact_pts.cols());

    #ifdef BULLET_COLLISION
    if (supp.contact_surface!=-1 && pdata->multi_robot) {
      // do bullet rigid body collision check

      auto multi_robot = static_cast<RigidBodyManipulator*>(pdata->multi_robot);
      
      if (supp.contact_surface < -1) {
        // check entire world
        multi_robot->getPointCollision(supp.body_idx,*pt_iter,pos,posB,normal);
      }
      else {
        // do pairwise check with specified contact surface
        multi_robot->getPairwisePointCollision(supp.body_idx,supp.contact_surface,*pt_iter,pos,posB,normal);
      }
      
      pos = posB-pos; // now -rel_pos in matlab version
    }
    else {
    #endif
      tmp = b->contact_pts.col(*pt_iter);
      pdata->r->forwardKin(supp.body_idx,tmp,0,contact_pos);
      collisionDetect(pdata->map_ptr,contact_pos,pos,NULL,terrain_height);
      pos -= contact_pos;  // now -rel_pos in matlab version
    #ifdef BULLET_COLLISION
    }
    #endif
  
		phi(i) = pos.norm();
		if (pos.dot(normal)>0)
			phi(i)=-phi(i);
		i++;
  }
	return nc;
}

int contactConstraints(struct QPControllerData* pdata, int nc, vector<SupportStateElement>& supp, MatrixXd &n, MatrixXd &D, MatrixXd &Jp, MatrixXd &Jpdot,double terrain_height)
{
  int j, k=0, nq = pdata->r->num_dof;

  n.resize(nc,nq);
  D.resize(nq,nc*2*m_surface_tangents);
  Jp.resize(3*nc,nq);
  Jpdot.resize(3*nc,nq);
  
  Vector3d contact_pos,pos,posB,normal; Vector4d tmp;
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  
  for (vector<SupportStateElement>::iterator iter = supp.begin(); iter!=supp.end(); iter++) {
    RigidBody* b = &(pdata->r->bodies[iter->body_idx]);
    if (nc>0) {
      for (set<int>::iterator pt_iter=iter->contact_pt_inds.begin(); pt_iter!=iter->contact_pt_inds.end(); pt_iter++) {
      	if (*pt_iter<0 || *pt_iter>=b->contact_pts.cols()) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","requesting contact pt %d but body only has %d pts",*pt_iter,b->contact_pts.cols());
        tmp = b->contact_pts.col(*pt_iter);
        pdata->r->forwardKin(iter->body_idx,tmp,0,contact_pos);
        pdata->r->forwardJac(iter->body_idx,tmp,0,J);

        #ifdef BULLET_COLLISION
        if (iter->contact_surface!=-1 && pdata->multi_robot) {
          // do bullet rigid body collision check
          auto multi_robot = static_cast<RigidBodyManipulator*>(pdata->multi_robot);

          if (iter->contact_surface < -1) {
            // check entire world
            multi_robot->getPointCollision(iter->body_idx,(int)*pt_iter,pos,posB,normal);
          }
          else {
            // do pairwise check with specified contact surface
            multi_robot->getPairwisePointCollision(iter->body_idx,iter->contact_surface,*pt_iter,pos,posB,normal);
          }
        }
        else {
        #endif
          collisionDetect(pdata->map_ptr,contact_pos,pos,&normal,terrain_height);
        #ifdef BULLET_COLLISION
        }
        #endif

        surfaceTangents(normal,d);

        n.row(k) = normal.transpose()*J;
        for (j=0; j<m_surface_tangents; j++) {
          D.col(2*k*m_surface_tangents+j) = J.transpose()*d.col(j);
          D.col((2*k+1)*m_surface_tangents+j) = -D.col(2*k*m_surface_tangents+j);
        }

        // store away kin sols into Jp and Jpdot
        // NOTE: I'm cheating and using a slightly different ordering of J and Jdot here
        Jp.block(3*k,0,3,nq) = J;
        pdata->r->forwardJacDot(iter->body_idx,tmp,0,J);
        Jpdot.block(3*k,0,3,nq) = J;
        
        k++;
      }
    }
  }
  
  return k;
}


int contactConstraintsBV(struct QPControllerData* pdata, int nc, double mu, vector<SupportStateElement>& supp, MatrixXd &B, MatrixXd &JB, MatrixXd &Jp, MatrixXd &Jpdot,double terrain_height)
{
  int j, k=0, nq = pdata->r->num_dof;

  B.resize(3,nc*2*m_surface_tangents);
  JB.resize(nq,nc*2*m_surface_tangents);
  Jp.resize(3*nc,nq);
  Jpdot.resize(3*nc,nq);
  
  Vector3d contact_pos,pos,posB,normal; Vector4d tmp;
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  double norm = sqrt(1+mu*mu); // because normals and ds are orthogonal, the norm has a simple form
  
  for (vector<SupportStateElement>::iterator iter = supp.begin(); iter!=supp.end(); iter++) {
    RigidBody* b = &(pdata->r->bodies[iter->body_idx]);
    if (nc>0) {
      for (set<int>::iterator pt_iter=iter->contact_pt_inds.begin(); pt_iter!=iter->contact_pt_inds.end(); pt_iter++) {
      	if (*pt_iter<0 || *pt_iter>=b->contact_pts.cols()) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","requesting contact pt %d but body only has %d pts",*pt_iter,b->contact_pts.cols());
        tmp = b->contact_pts.col(*pt_iter);
        pdata->r->forwardKin(iter->body_idx,tmp,0,contact_pos);
        pdata->r->forwardJac(iter->body_idx,tmp,0,J);

        #ifdef BULLET_COLLISION
        if (iter->contact_surface!=-1 && pdata->multi_robot) {
          // do bullet rigid body collision check
          auto multi_robot = static_cast<RigidBodyManipulator*>(pdata->multi_robot);

          if (iter->contact_surface < -1) {
            // check entire world
            multi_robot->getPointCollision(iter->body_idx,(int)*pt_iter,pos,posB,normal);
          }
          else {
            // do pairwise check with specified contact surface
            multi_robot->getPairwisePointCollision(iter->body_idx,iter->contact_surface,*pt_iter,pos,posB,normal);
          }
        }
        else {
        #endif
          collisionDetect(pdata->map_ptr,contact_pos,pos,&normal,terrain_height);
        #ifdef BULLET_COLLISION
        }
        #endif

        surfaceTangents(normal,d);
        for (j=0; j<m_surface_tangents; j++) {
          B.col(2*k*m_surface_tangents+j) = (normal + mu*d.col(j)) / norm; 
          B.col((2*k+1)*m_surface_tangents+j) = (normal - mu*d.col(j)) / norm; 
    
          JB.col(2*k*m_surface_tangents+j) = J.transpose()*B.col(2*k*m_surface_tangents+j);
          JB.col((2*k+1)*m_surface_tangents+j) = J.transpose()*B.col((2*k+1)*m_surface_tangents+j);
        }

        // store away kin sols into Jp and Jpdot
        // NOTE: I'm cheating and using a slightly different ordering of J and Jdot here
        Jp.block(3*k,0,3,nq) = J;
        pdata->r->forwardJacDot(iter->body_idx,tmp,0,J);
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
  if (nrhs<1) mexErrMsgTxt("usage: ptr = QPControllermex(0,control_obj,robot_obj,...); alpha=QPControllermex(ptr,...,...)");
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

    pdata->B_act.resize(nu,nu);
    pdata->B_act = pdata->B.bottomRows(nu);

     // get the map ptr back from matlab
     if (!mxIsNumeric(prhs[6]) || mxGetNumberOfElements(prhs[6])!=1)
     mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the seventh argument should be the map ptr");
     memcpy(&pdata->map_ptr,mxGetPr(prhs[6]),sizeof(pdata->map_ptr));
    
//    pdata->map_ptr = NULL;
    if (!pdata->map_ptr)
      mexWarnMsgTxt("Map ptr is NULL.  Assuming flat terrain at z=0");
    
    // get the multi-robot ptr back from matlab
    if (!mxIsNumeric(prhs[7]) || mxGetNumberOfElements(prhs[7])!=1)
    mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the eigth argument should be the multi_robot ptr");
    memcpy(&pdata->multi_robot,mxGetPr(prhs[7]),sizeof(pdata->multi_robot));

    // create gurobi environment
    error = GRBloadenv(&(pdata->env),NULL);

    // set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
    mxArray* psolveropts = myGetProperty(pobj,"solver_options");
    int method = (int) mxGetScalar(myGetField(psolveropts,"method"));
    CGE ( GRBsetintparam(pdata->env,"outputflag",0), pdata->env );
    CGE ( GRBsetintparam(pdata->env,"method",2), pdata->env );
    CGE ( GRBsetintparam(pdata->env,"presolve",0), pdata->env );
    if (method==2) {
    	CGE ( GRBsetintparam(pdata->env,"bariterlimit",20), pdata->env );
    	CGE ( GRBsetintparam(pdata->env,"barhomogeneous",0), pdata->env );
    	CGE ( GRBsetdblparam(pdata->env,"barconvtol",0.0005), pdata->env );
    }

    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:constructModelmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));
    
    // preallocate some memory
    pdata->H.resize(nq,nq);
    pdata->H_float.resize(6,nq);
    pdata->H_act.resize(nu,nq);

    pdata->C.resize(nq);
    pdata->C_float.resize(6);
    pdata->C_act.resize(nu);

    pdata->J.resize(3,nq);
    pdata->Jdot.resize(3,nq);
    pdata->J_xy.resize(2,nq);
    pdata->Jdot_xy.resize(2,nq);
    pdata->Hqp.resize(nq,nq);
    pdata->fqp.resize(nq);
    pdata->Ag.resize(6,nq);
    pdata->Agdot.resize(6,nq);
    return;
  }
  
  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:QPControllermex:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

//  for (i=0; i<pdata->r->num_bodies; i++)
//  	mexPrintf("body %d (%s) has %d contact points\n", i, pdata->r->bodies[i].linkname.c_str(), pdata->r->bodies[i].contact_pts.cols());

  int nu = pdata->B.cols(), nq = pdata->r->num_dof;
  const int dim = 3, // 3D
      nd = 2*m_surface_tangents; // for friction cone approx, hard coded for now
  
  assert(nu+6 == nq);

  int narg=1;

  int use_fast_qp = (int) mxGetScalar(prhs[narg++]);
  
  Map< VectorXd > q_ddot_des(mxGetPr(prhs[narg++]),nq);
  
  double *q = mxGetPr(prhs[narg++]);
  double *qd = &q[nq];
  double *q_multi = mxGetPr(prhs[narg++]);
  
  int desired_support_argid = narg++;
  
  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==4);
  Map< Matrix4d > A_ls(mxGetPr(prhs[narg++]));

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
  Map< Vector4d > s1dot(mxGetPr(prhs[narg++]));

  double s2dot = mxGetScalar(prhs[narg++]);

  assert(mxGetM(prhs[narg])==4); assert(mxGetN(prhs[narg])==1);
  Map< Vector4d > x0(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==1);
  Map< Vector2d > u0(mxGetPr(prhs[narg++]));

  assert(mxGetM(prhs[narg])==2); assert(mxGetN(prhs[narg])==1);
  Map< Vector2d > y0(mxGetPr(prhs[narg++]));

  double mu = mxGetScalar(prhs[narg++]);

  double* double_contact_sensor = mxGetPr(prhs[narg]); int len = mxGetNumberOfElements(prhs[narg++]);
  VectorXi contact_sensor(len);  
  for (i=0; i<len; i++)
    contact_sensor(i)=(int)double_contact_sensor[i];
  double contact_threshold = mxGetScalar(prhs[narg++]);
  double terrain_height = mxGetScalar(prhs[narg++]); // nonzero if we're using DRCFlatTerrainMap
  int include_angular_momentum = (int) mxGetScalar(prhs[narg++]); 

  Matrix2d R_DQyD_ls = R_ls + D_ls.transpose()*Qy*D_ls;
  
  pdata->r->doKinematics(q,false,qd);

  #ifdef BULLET_COLLISION
  if (pdata->multi_robot) {
    auto multi_robot = static_cast<RigidBodyManipulator*>(pdata->multi_robot);
    multi_robot->doKinematics(q_multi,false);
  }
  #endif
  
  //---------------------------------------------------------------------
  // Compute active support from desired supports -----------------------

  vector<SupportStateElement> active_supports;
  int num_active_contact_pts=0;
  if (!mxIsEmpty(prhs[desired_support_argid])) {
    VectorXd phi;
    mxArray* mxBodies = mxGetProperty(prhs[desired_support_argid],0,"bodies");
    if (!mxBodies) mexErrMsgTxt("couldn't get bodies");
    double* pBodies = mxGetPr(mxBodies);
    mxArray* mxContactPts = mxGetProperty(prhs[desired_support_argid],0,"contact_pts");
    if (!mxContactPts) mexErrMsgTxt("couldn't get contact points");
    mxArray* mxContactSurfaces = mxGetProperty(prhs[desired_support_argid],0,"contact_surfaces");
    if (!mxContactSurfaces) mexErrMsgTxt("couldn't get contact surfaces");
    double* pContactSurfaces = mxGetPr(mxContactSurfaces);
    
    for (i=0; i<mxGetNumberOfElements(mxBodies);i++) {
      mxArray* mxBodyContactPts = mxGetCell(mxContactPts,i);
      int nc = mxGetNumberOfElements(mxBodyContactPts);
      if (nc<1) continue;
      
      SupportStateElement se;
      se.body_idx = (int) pBodies[i]-1;
      pr = mxGetPr(mxBodyContactPts); 
      for (j=0; j<nc; j++) {
//      	mexPrintf("adding pt %d to body %d\n", (int)pr[j]-1, se.body_idx);
        se.contact_pt_inds.insert((int)pr[j]-1);
      }
      se.contact_surface = (int) pContactSurfaces[i]-1;
      
      if (contact_threshold == -1) { // ignore terrain
        if (contact_sensor(i)!=0) { // no sensor info, or sensor says yes contact
          active_supports.push_back(se);
          num_active_contact_pts += nc;
        }
      } else {
        contactPhi(pdata,se,phi,terrain_height);
        if (phi.minCoeff()<=contact_threshold || contact_sensor(i)==1) { // any contact below threshold (kinematically) OR contact sensor says yes contact
          active_supports.push_back(se);
          num_active_contact_pts += nc;
        }
      }
    }
  }

  pdata->r->HandC(q,qd,(MatrixXd*)NULL,pdata->H,pdata->C,(MatrixXd*)NULL,(MatrixXd*)NULL);
  pdata->H_float = pdata->H.topRows(6);
  pdata->H_act = pdata->H.bottomRows(nu);
  pdata->C_float = pdata->C.head(6);
  pdata->C_act = pdata->C.tail(nu);
 
  MatrixXd Ag_ang,Agdot_ang;
  Vector3d h_ang_dot_des; 
  double w2 = 0.0001; // QP angular momentum objective weight
  if (include_angular_momentum == 1) {
    pdata->r->getCMM(q,qd,pdata->Ag,pdata->Agdot);
    Ag_ang = pdata->Ag.topRows(3);
    Agdot_ang = pdata->Agdot.topRows(3);
    h_ang_dot_des = Vector3d::Zero(); // regulate to zero for now
  }
  
  Vector3d xcom;
  // consider making all J's into row-major
  
  pdata->r->getCOM(xcom);
  pdata->r->getCOMJac(pdata->J);
  pdata->r->getCOMJacDot(pdata->Jdot);
  // copy to xy versions for computations below (avoid doing topRows a bunch of times)
  pdata->J_xy = pdata->J.topRows(2);
  pdata->Jdot_xy = pdata->Jdot.topRows(2);

  Map<VectorXd> qdvec(qd,nq);
  
  MatrixXd Jz,Jp,Jpdot,D;
  int nc = contactConstraintsBV(pdata,num_active_contact_pts,mu,active_supports,Jz,D,Jp,Jpdot,terrain_height);
  int neps = nc*dim;

  Vector4d x_bar,xlimp;
  MatrixXd D_float(6,D.cols()), D_act(nu,D.cols());
  if (nc>0) {
    xlimp << xcom.topRows(2),pdata->J_xy*qdvec;
    x_bar = xlimp-x0;

    D_float = D.topRows(6);
    D_act = D.bottomRows(nu);
  }

  int nf = nc*nd; // number of contact force variables
  int nparams = nq+nf+neps;
  
  //----------------------------------------------------------------------
  // QP cost function ----------------------------------------------------
  //
  //  min: quad(Jdot*qd + J*qdd,R_ls)+quad(C*x+D*(Jdot*qd + J*qdd),Qy) + (2*x'*S + s1')*(A*x + B*(Jdot*qd + J*qdd)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)
  VectorXd f(nparams);
  {      
    if (nc > 0) {
      // NOTE: moved Hqp calcs below, because I compute the inverse directly for FastQP (and sparse Hqp for gurobi)

      pdata->fqp = (C_ls*xlimp).transpose()*Qy*D_ls*pdata->J_xy;
      pdata->fqp += (pdata->Jdot_xy*qdvec).transpose()*R_DQyD_ls*pdata->J_xy;
      pdata->fqp += (S*x_bar + 0.5*s1).transpose()*B_ls*pdata->J_xy;
      pdata->fqp -= u0.transpose()*R_DQyD_ls*pdata->J_xy;
      pdata->fqp -= y0.transpose()*Qy*D_ls*pdata->J_xy;
      pdata->fqp -= pdata->w*q_ddot_des.transpose();
      if (include_angular_momentum == 1) {
        pdata->fqp += w2*qdvec.transpose()*Agdot_ang.transpose()*Ag_ang;
        pdata->fqp -= w2*h_ang_dot_des.transpose()*Ag_ang;
      }
     
      // obj(1:nq) = fqp
      f.head(nq) = pdata->fqp.transpose();
     } else {
      // obj(1:nq) = -qddot_des
      f.head(nq) = -q_ddot_des;
    } 
  }
  f.tail(nf+neps) = VectorXd::Zero(nf+neps);

  MatrixXd Aeq(6+neps,nparams);
  Aeq.topRightCorner(6+neps,neps) = MatrixXd::Zero(6+neps,neps);  // note: obvious sparsity here
  VectorXd beq(6+neps);
  
  // constrained floating base dynamics
  //  H_float*qdd - J_float'*lambda - Dbar_float*beta = -C_float
  Aeq.topLeftCorner(6,nq) = pdata->H_float;
  beq.topRows(6) = -pdata->C_float;
    
  if (nc>0) {
    Aeq.block(0,nq,6,nc*nd) = -D_float;
  }
  
  if (nc > 0) {
    // relative acceleration constraint
    Aeq.bottomLeftCorner(neps,nq) = Jp;
    Aeq.block(6,nq,neps,nf) = MatrixXd::Zero(neps,nf);  // note: obvious sparsity here
    Aeq.bottomRightCorner(neps,neps) = MatrixXd::Identity(neps,neps);             // note: obvious sparsity here
    beq.bottomRows(neps) = (-Jpdot - 1.0*Jp)*qdvec;
  }    

  
  MatrixXd Ain = MatrixXd::Zero(2*nu,nparams);  // note: obvious sparsity here
  VectorXd bin = VectorXd::Zero(2*nu);

  // linear input saturation constraints
  // u=B_act'*(H_act*qdd + C_act - Jz_act'*z - Dbar_act*beta)
  // using transpose instead of inverse because B is orthogonal
  Ain.topLeftCorner(nu,nq) = pdata->B_act.transpose()*pdata->H_act;
  Ain.block(0,nq,nu,nc*nd) = -pdata->B_act.transpose()*D_act;
  bin.head(nu) = -pdata->B_act.transpose()*pdata->C_act + pdata->umax;

  Ain.block(nu,0,nu,nparams) = -1*Ain.block(0,0,nu,nparams);
  bin.segment(nu,nu) = pdata->B_act.transpose()*pdata->C_act - pdata->umin;


  GRBmodel * model = NULL;
  int info=-1;
  
  // set obj,lb,up
  VectorXd lb(nparams), ub(nparams);
  lb.head(nq) = -1e3*VectorXd::Ones(nq);
  ub.head(nq) = 1e3*VectorXd::Ones(nq);
  lb.segment(nq,nf) = VectorXd::Zero(nf);
  ub.segment(nq,nf) = 500*VectorXd::Ones(nf);
  lb.tail(neps) = -pdata->slack_limit*VectorXd::Ones(neps);
  ub.tail(neps) = pdata->slack_limit*VectorXd::Ones(neps);

  for (int jj=0; jj<pdata->r->num_dof; jj++) {
    // set acceleration limit to zero if at joint limit
    if (q[jj] >= pdata->r->joint_limit_max[jj]-1e-4)
      ub(jj) = 0;
    if (q[jj] <= pdata->r->joint_limit_min[jj]+1e-4)
      lb(jj) = 0;
  }
  
  VectorXd alpha(nparams);

  MatrixXd Qnfdiag(nf,1), Qneps(neps,1);
  vector< MatrixXd* > QBlkDiag( nc>0 ? 3 : 1 );  // nq, nf, neps   // this one is for gurobi

  if (use_fast_qp > 0 && include_angular_momentum == 0)
  { // set up and call fastqp

  	//    We want Hqp inverse, which I can compute efficiently using the
  	//    matrix inversion lemma (see wikipedia):
  	//    inv(A + U'CV) = inv(A) - inv(A)*U* inv([ inv(C)+ V*inv(A)*U ]) V inv(A)
  	//     but inv(A) is 1/w*eye(nq), so I can reduce this to:
  	//        = 1/w ( eye(nq) - 1/w*U* inv[ inv(C) + 1/w*V*U ] * V
  	if (nc>0) {
  		double wi = 1/(pdata->w + REG);
			pdata->Hqp = wi*MatrixXd::Identity(nq,nq);
  		if (R_DQyD_ls.trace()>1e-15) // R_DQyD_ls is not zero
  			pdata->Hqp -=  wi*wi*pdata->J_xy.transpose()*(R_DQyD_ls.inverse() + wi*pdata->J_xy*pdata->J_xy.transpose()).inverse()*pdata->J_xy;
  	} else {
    	pdata->Hqp = MatrixXd::Constant(nq,1,1/(1+REG));
  	}

	#ifdef TEST_FAST_QP
  	MatrixXd Hqp_test(nq,nq) = (pdata->J_xy.transpose()*R_DQyD_ls*pdata->J_xy + (pdata->w+REG)*MatrixXd::Identity(nq,nq)).inverse();
  	if (((Hqp_test-pdata->Hqp).abs()).maxCoeff() > 1e-6)
  		mexErrMsgIdAndTxt("Q submatrix inverse from matrix inversion lemma does not match direct Q inverse.");
	#endif

    Qnfdiag = MatrixXd::Constant(nf,1,1/REG);
    Qneps = MatrixXd::Constant(neps,1,1/(.001+REG));

    QBlkDiag[0] = &pdata->Hqp;
    if (nc>0) {
    	QBlkDiag[1] = &Qnfdiag;
    	QBlkDiag[2] = &Qneps;     // quadratic slack var cost, Q(nparams-neps:end,nparams-neps:end)=eye(neps)
    }

    MatrixXd Ain_lb_ub(2*nu+2*nparams,nparams);
    VectorXd bin_lb_ub(2*nu+2*nparams);
    Ain_lb_ub << Ain, 			     // note: obvious sparsity here
    		-MatrixXd::Identity(nparams,nparams),
    		MatrixXd::Identity(nparams,nparams);
    bin_lb_ub << bin, -lb, ub;

    info = fastQPThatTakesQinv(QBlkDiag, f, Aeq, beq, Ain_lb_ub, bin_lb_ub, pdata->active, alpha);

    if (info<0)  	mexPrintf("fastQP info = %d.  Calling gurobi.\n", info);
  }

  if (info<0) {
		// now set up gurobi:
    if (nc > 0) {
      pdata->Hqp = pdata->J_xy.transpose()*R_DQyD_ls*pdata->J_xy;          // note: only needed for gurobi call (could pull it down)
      pdata->Hqp += (pdata->w+REG)*MatrixXd::Identity(nq,nq);
      if (include_angular_momentum == 1)
        pdata->Hqp += w2*Ag_ang.transpose()*Ag_ang;
                 
    } else {
      // Q(1:nq,1:nq) = eye(nq)
    	pdata->Hqp = MatrixXd::Constant(nq,1,1+REG);
    }

    Qnfdiag = MatrixXd::Constant(nf,1,REG);
    Qneps = MatrixXd::Constant(neps,1,.001+REG);

    QBlkDiag[0] = &pdata->Hqp;
    if (nc>0) {
    	QBlkDiag[1] = &Qnfdiag;
    	QBlkDiag[2] = &Qneps;     // quadratic slack var cost, Q(nparams-neps:end,nparams-neps:end)=eye(neps)
    }

		model = gurobiQP(pdata->env,QBlkDiag,f,Aeq,beq,Ain,bin,lb,ub,pdata->active,alpha);

	  int status; CGE ( GRBgetintattr(model, "Status", &status) , pdata->env);
	  if (status!=2) mexPrintf("gurobi reports non-optimal status = %d\n", status);
  }


  //----------------------------------------------------------------------
  // Solve for inputs ----------------------------------------------------
  VectorXd y(nu);
  VectorXd qdd = alpha.head(nq);
  VectorXd beta = alpha.segment(nq,nc*nd);

  // use transpose because B_act is orthogonal
  y = pdata->B_act.transpose()*(pdata->H_act*qdd + pdata->C_act - D_act*beta);
  //y = pdata->B_act.jacobiSvd(ComputeThinU|ComputeThinV).solve(pdata->H_act*qdd + pdata->C_act - Jz_act.transpose()*lambda - D_act*beta);
  
  if (nlhs>0) plhs[0] = eigenToMatlab(y);

  if (nlhs>1) {
    double Vdot;
    if (nc>0) 
      // note: Sdot is 0 for ZMP/double integrator dynamics, so we omit that term here
      Vdot = ((2*x_bar.transpose()*S + s1.transpose())*(A_ls*x_bar + B_ls*(pdata->Jdot_xy*qdvec + pdata->J_xy*qdd)) + s1dot.transpose()*x_bar)(0) + s2dot;
    else
      Vdot = 0;
    plhs[1] = mxCreateDoubleScalar(Vdot);
  }

  if (nlhs>2) {
      plhs[2] = mxCreateDoubleMatrix(1,active_supports.size(),mxREAL);
      pr = mxGetPr(plhs[2]);
      int i=0;
      for (vector<SupportStateElement>::iterator iter = active_supports.begin(); iter!=active_supports.end(); iter++) {
          pr[i++] = (double) (iter->body_idx + 1);
      }
  }

  if (nlhs>3) {
    plhs[3] = eigenToMatlab(qdd);
  }

  if (model) {  // todo: return more info for fastQP

		if (nlhs>3) {  // return model.Q (for unit testing)
			int qnz;
			CGE (GRBgetintattr(model,"NumQNZs",&qnz), pdata->env);
			int *qrow = new int[qnz], *qcol = new int[qnz];
			double* qval = new double[qnz];
			CGE (GRBgetq(model,&qnz,qrow,qcol,qval), pdata->env);
			plhs[3] = mxCreateDoubleMatrix(nparams,nparams,mxREAL);
			double* pm = mxGetPr(plhs[3]);
			memset(pm,0,sizeof(double)*nparams*nparams);
			for (i=0; i<qnz; i++)
				pm[qrow[i]+nparams*qcol[i]] = qval[i];
			delete[] qrow;
			delete[] qcol;
			delete[] qval;

			if (nlhs>4) {  // return model.obj (for unit testing)
				plhs[4] = mxCreateDoubleMatrix(1,nparams,mxREAL);
				CGE (GRBgetdblattrarray(model, "Obj", 0, nparams, mxGetPr(plhs[4])), pdata->env);

				if (nlhs>5) {  // return model.A (for unit testing)
					int numcon;
					CGE (GRBgetintattr(model,"NumConstrs",&numcon), pdata->env);
					plhs[5] = mxCreateDoubleMatrix(numcon,nparams,mxREAL);
					double *pm = mxGetPr(plhs[5]);
					for (i=0; i<numcon; i++)
						for (j=0; j<nparams; j++)
							CGE (GRBgetcoeff(model,i,j,&pm[i+j*numcon]), pdata->env);

					if (nlhs>6) {  // return model.rhs (for unit testing)
						plhs[6] = mxCreateDoubleMatrix(numcon,1,mxREAL);
						CGE (GRBgetdblattrarray(model,"RHS",0,numcon,mxGetPr(plhs[6])), pdata->env);
					}

					if (nlhs>7) { // return model.sense
						char* sense = new char[numcon+1];
						CGE (GRBgetcharattrarray(model,"Sense",0,numcon,sense), pdata->env);
						sense[numcon]='\0';
						plhs[7] = mxCreateString(sense);
						// delete[] sense;  // it seems that I'm not supposed to free this
					}

					if (nlhs>8) {
						plhs[8] = mxCreateDoubleMatrix(nparams,1,mxREAL);
						CGE (GRBgetdblattrarray(model, "LB", 0, nparams, mxGetPr(plhs[8])), pdata->env);
					}
					if (nlhs>9) {
						plhs[9] = mxCreateDoubleMatrix(nparams,1,mxREAL);
						CGE (GRBgetdblattrarray(model, "UB", 0, nparams, mxGetPr(plhs[9])), pdata->env);
					}
				}
			}
		}

		GRBfreemodel(model);
  }
  
  //  GRBfreeenv(env);
} 
