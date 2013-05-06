/* 
 * A c++ version of (significant pieces of) the QPController.m mimoOutput method. 
 *
 */

#include <mex.h>
#include <gurobi_c++.h>

//struct QPControllerData {
//  
//};


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  int error;
  if (nrhs<1) mexErrMsgTxt("usage: QPControllermex(model)");
  const mxArray *pmodel = prhs[0];
  
  const mxArray *Q = mxGetField(pmodel,0,"Q"),
          *obj = mxGetField(pmodel,0,"obj"),
          *A = mxGetField(pmodel,0,"A"),
          *rhs = mxGetField(pmodel,0,"rhs"),
          *sense = mxGetField(pmodel,0,"sense"),
          *lb = mxGetField(pmodel,0,"lb"),
          *ub = mxGetField(pmodel,0,"ub");
  
  if (!Q || !obj || !A || !rhs || !sense || !lb || !ub)
    mexErrMsgTxt("oops.  didn't get data out of structure properly");

  int nx = mxGetN(A), i;

  GRBenv *env = NULL;  
  error = GRBloadenv(&env,NULL);

  // set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
  error = GRBsetintparam(env,"outputflag",0);
  error = GRBsetintparam(env,"method",2);
  error = GRBsetintparam(env,"presolve",0);
  error = GRBsetintparam(env,"bariterlimit",20);
  error = GRBsetintparam(env,"barhomogenous",0);
  error = GRBsetdblparam(env,"barconvtol",0.0005);
  
  GRBmodel *model = NULL;
  // set obj,lb,up
  error = GRBnewmodel(env,&model,"QPController",nx,mxGetPr(obj),mxGetPr(lb),mxGetPr(ub),NULL,NULL);
    
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
  GRBfreeenv(env);
  mxDestroyArray(At);
} 
