#include "mex.h"
#include "ConstraintApp.h"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  if ( nrhs != 1 ) {
    mexErrMsgTxt("requires exactly 1 parameters"); 
  }

  if ( nlhs != 1 ) {
    mexErrMsgTxt("returns exactly 1 parameters"); 
  }

  ConstraintApp* app(*(ConstraintApp**)mxGetData(prhs[0]));

  std::vector<double> state;
  app->GetCurrentStateEstimate(state);

  plhs[0] = mxCreateDoubleMatrix(1, state.size(), mxREAL);

  double* s((double*)mxGetData(plhs[0]));

  std::copy(state.begin(), state.end(), s);
}
