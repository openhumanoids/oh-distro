
// simple mex function for solving IK problems as equality constrained QPs 

#include "QPCommon.h"
#include <Eigen/StdVector>

void angleDiff(VectorXd phi1, VectorXd phi2, VectorXd* d) {
  *d = phi2 - phi1;
  
  for (int i = 0; i < phi1.size(); i++) {
    if ((*d)(i) < -M_PI) {
      (*d)(i) = fmod((*d)(i) + M_PI,2*M_PI) + M_PI;
    } else {
      (*d)(i) = fmod((*d)(i) + M_PI, 2*M_PI) - M_PI;
    }
  }
}

void angleDiff(MatrixXd phi1, MatrixXd phi2, MatrixXd* d) {
  *d = phi2 - phi1;
  
  for (int i = 0; i < phi1.rows(); i++) {
    for (int j = 0; j < phi1.cols(); j++) {
      if ((*d)(i,j) < -M_PI) {
        (*d)(i,j) = fmod((*d)(i,j) + M_PI, 2*M_PI) + M_PI;
      } else {
        (*d)(i,j) = fmod((*d)(i,j) + M_PI, 2*M_PI) - M_PI;
      }
    }
  }
}

void* getDrakeMexPointer(const mxArray* mx)
{
  void* ptr = NULL;

  // todo: optimize this by caching the pointer values, as described in
  // http://groups.csail.mit.edu/locomotion/bugs/show_bug.cgi?id=1590
  mxArray* ptrArray = mxGetProperty(mx,0,"ptr");
  if (!ptrArray)
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs","cannot retrieve 'ptr' field from this mxArray.  are you sure it's a valid DrakeMexPointer object?");

  if (!mxIsNumeric(ptrArray) || mxGetNumberOfElements(ptrArray)!=1)
    mexErrMsgIdAndTxt("Drake:getDrakeMexPointer:BadInputs","the ptr property of this DrakeMexPointer does not appear to contain a valid pointer");
  memcpy(&ptr,mxGetData(ptrArray),sizeof(ptr));     // note: could use a reinterpret_cast here instead

  return ptr;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs < 3) {
    mexErrMsgIdAndTxt("linearIKmex:NotEnoughInputs","Usage linearIKmex(model_ptr,q0,q_nom,Q,...)");
  }
  
  if (nlhs<1) return;

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int nq = model->num_dof;

  // set up cost function
  // cost:  (q-q_nom)'*Q*(q-q_nom)  \equiv q'*Q*q - 2*q_nom'*Q*q  (const term doesn't matter)
  Map<VectorXd> q_nom(mxGetPr(prhs[2]),nq);
  Map<MatrixXd> Q(mxGetPr(prhs[3]),nq,nq);
  VectorXd f = -2*Q*q_nom;
  
  double *q0 = mxGetPr(prhs[1]);
  model->doKinematics(q0);
  
  VectorXd q0vec = Map<VectorXd>(q0,nq);

  std::vector<VectorXd,aligned_allocator<VectorXd>> Aeq_ (nrhs*6); // nrhs*6 is an upper bound
  VectorXd beq_ = VectorXd::Zero(nrhs*6); // nrhs*6 is an upper bound
  int eq_count = 0;
  
  MatrixXd body_pos;
  body_pos.resize(4,1); 
  VectorXd world_pos;
  
  int i=4;
  while (i<nrhs) {
    int rows;
  	int body_ind = mxGetScalar(prhs[i]) - 1;
    mxArray *min = NULL;
    mxArray *max = NULL;

    if (mxGetN(prhs[i+1]) != 1) {
      mexErrMsgIdAndTxt("linearIKmex:BadInputs", "num pts must be 1");
    }

    if (body_ind==-1) {
      body_pos << 0,0,0,1;
      world_pos = Map<VectorXd>(mxGetPr(prhs[i+1]),3);
      rows = 3;
      i+=2;
    } 
    else {
      if (mxIsClass(prhs[i+2],"struct"))  {//isstruct(worldpos)
        min = mxGetField(prhs[i+2],0,"min");
        max = mxGetField(prhs[i+2],0,"max");
        
        if (min == NULL || max == NULL) {
          mexErrMsgIdAndTxt("linearIKmex:BadInputs", "if world_pos is a struct, it must have fields .min and .max");
        }
        
        rows = mxGetM(min);        
        if (rows != 3 && rows != 6) {
          mexErrMsgIdAndTxt("linearIKmex:BadInputs", "world_pos.min must have 3 or 6 rows");
        }

        if (mxGetM(max) != rows) {
          mexErrMsgIdAndTxt("linearIKmex:BadInputs", "world_pos.max must have the same number of rows as world_pos.min");
        }
      } 
      else {

        rows = mxGetM(prhs[i+2]);
        world_pos = Map<VectorXd>(mxGetPr(prhs[i+2]),rows);
      }

      if (mxIsClass(prhs[i+1],"char") || mxGetM(prhs[i+1]) == 1) {
        mexErrMsgIdAndTxt("linearIKmex:NotImplemented", "collision group not implemented in mex");
      } 
      else {
        if (mxGetM(prhs[i+1]) !=3) {
          mexErrMsgIdAndTxt("linearIKmex:BadInputs", "bodypos must be 3x1");
        }

        Map<VectorXd> pt_tmp(mxGetPr(prhs[i+1]), 3);                    
        body_pos << pt_tmp, 1;
      }
      i+=3;
    }   
    
    MatrixXd x;
    MatrixXd J;
    if (body_ind == -1) {
      x = MatrixXd::Zero(3,1);
      Vector3d x_com;
      
      model->getCOM(x_com);      
      model->getCOMJac(J);     
      x.resize(3,1);
      x << x_com;      
      
    } 
    else {
      J.resize(rows,nq);
      x.resize(rows,1);

      model->forwardKin(body_ind, body_pos, (int) rows==6, x);
      model->forwardJac(body_ind, body_pos, (int) rows==6, J);

      if (rows == 6 && min == NULL && max == NULL) {
        VectorXd delta;
        angleDiff(x.block(3,0,3,1), world_pos.block(3,0,3,1), &delta);
        world_pos.block(3,0,3,1) = x.block(3,0,3,1)+delta;
      }
    }
      
    if (min == NULL && max == NULL) {
      for (int j = 0; j < rows; j++) {
        if (!mxIsNaN(world_pos(j))) {
          Aeq_[eq_count] = J.row(j);
          beq_(eq_count++) = world_pos(j) - x(j) + J.row(j)*q0vec;   
        }
      } 
    }
    else {
      mexErrMsgIdAndTxt("linearIKmex:BadInputs", "only equality constraints are supported");
    }
  }
  
  MatrixXd Aeq = MatrixXd::Zero(eq_count,nq);
  VectorXd beq = beq_.head(eq_count);
  for (int j = 0; j < eq_count; j++) {
    Aeq.row(j) = Aeq_[j];
  }

  // todo: exploit diagonal Q and/or call fastqp :)

  MatrixXd A = MatrixXd::Zero(nq+eq_count,nq+eq_count);
  A.topLeftCorner(nq,nq) = Q;
  A.topRightCorner(nq,eq_count) = Aeq.transpose();
  A.bottomLeftCorner(eq_count,nq) = Aeq;

  VectorXd b = VectorXd::Zero(nq+eq_count);
  b.head(nq) = -0.5*f;
  b.tail(eq_count) = beq;

  // VectorXd y = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
  VectorXd y = A.colPivHouseholderQr().solve(b);

  VectorXd q = y.head(nq);
  plhs[0] = eigenToMatlab(q);
  
  // plhs[1] = eigenToMatlab(Aeq);
  
  // plhs[2] = eigenToMatlab(beq);
  
  // plhs[3] = eigenToMatlab(A);

  // plhs[4] = eigenToMatlab(b);
  
  return;
}
