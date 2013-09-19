#include "mex.h"
#include "ConstraintApp_MB.h"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  if ( nrhs != 4 ) {
    mexErrMsgTxt("requires exactly 4 parameters: (ptr, state, ids, method)"); 
  }

  if ( nlhs != 1 ) {
    mexErrMsgTxt("returns exactly 1 parameters"); 
  }

  ConstraintApp_MB* app(*(ConstraintApp_MB**)mxGetData(prhs[0]));

  int stateSize = app->GetStateSize();
  if ( mxGetM(prhs[1])*mxGetN(prhs[1]) != stateSize ) {
    mexErrMsgTxt("state must be Sx1, where S is the size of the state");
  }

  int idSize = mxGetNumberOfElements(prhs[2]);

  std::vector<std::string> ids(idSize);
  const mxArray* idCellArray = prhs[2];
  for ( int i = 0; i < idSize; i++ ) {
    const mxArray* string_array = mxGetCell(idCellArray, i);
    int buflen = mxGetNumberOfElements(string_array) + 1;
    char* str = new char[buflen];
    mxGetString(string_array, str, buflen);
    //mexPrintf("cam_getJacobian: id[%i]=[%s]\n", i, str);
    ids[i] = std::string(str);
    delete str;
  }

  double method = mxGetScalar(prhs[3]);

  double* s((double*)mxGetData(prhs[1]));
  ConstraintApp_MB::StateVector state(stateSize);
  for ( int i = 0; i < stateSize; i++ ) {
    state[i] = s[i];
  }

  ConstraintApp_MB::Jacobian jacobian;
  if ( !app->GetJacobian(state, ids, jacobian, (int)method) ) {
    mexErrMsgTxt("error while computing jacobian");
  }

  plhs[0] = mxCreateDoubleMatrix(jacobian.rows(), jacobian.cols(), mxREAL);
  double* jac((double*)mxGetData(plhs[0]));
  memcpy(jac, jacobian.data(), jacobian.rows()*jacobian.cols()*sizeof(double));
}
