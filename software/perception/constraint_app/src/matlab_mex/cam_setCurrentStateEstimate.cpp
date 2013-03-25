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

  if ( mxGetN(prhs[1]) * mxGetM(prhs[1]) != 6 ) {
    mexErrMsgTxt("state must be 6x1");
  }

  ConstraintApp* app(*(ConstraintApp**)mxGetData(prhs[0]));
  double* s((double*)mxGetData(prhs[1]));

  std::vector<double> state(6);
  std::copy(s, s+6, state.begin());
  app->SetCurrentStateEstimate(state);
}
