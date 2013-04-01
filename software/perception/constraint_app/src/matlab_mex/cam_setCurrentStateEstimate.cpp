#include "mex.h"
#include "ConstraintApp.h"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  if ( nrhs != 2 ) {
    mexErrMsgTxt("requires exactly 2 parameters"); 
  }

  if ( nlhs != 0 ) {
    mexErrMsgTxt("returns exactly 0 parameters"); 
  }

  ConstraintApp* app(*(ConstraintApp**)mxGetData(prhs[0]));

  int stateSize = app->GetStateSize();
  if ( mxGetN(prhs[1]) * mxGetM(prhs[1]) != stateSize ) {
    mexErrMsgTxt("state must be Sx1, where S is the state size");
  }

  double* s((double*)mxGetData(prhs[1]));

  std::vector<double> state(stateSize);
  std::copy(s, s+stateSize, state.begin());
  app->SetCurrentStateEstimate(state);
}
