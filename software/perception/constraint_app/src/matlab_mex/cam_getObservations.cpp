#include "mex.h"
#include "ConstraintApp.h"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  if ( nrhs != 1 ) {
    mexErrMsgTxt("requires exactly 1 parameters"); 
  }

  ConstraintApp* app(*(ConstraintApp**)mxGetData(prhs[0]));

  if ( nlhs != 3 ) {
    mexErrMsgTxt("returns exactly 3 parameters");
  }

  std::vector<double> expectedObservations;
  std::vector<double> actualObservations;
  bool res = app->GetObservations(expectedObservations, actualObservations);

  plhs[0] = mxCreateLogicalScalar(res);
  plhs[1] = mxCreateDoubleMatrix(1, expectedObservations.size(), mxREAL);
  plhs[2] = mxCreateDoubleMatrix(1, actualObservations.size(), mxREAL);

  double* eo((double*)mxGetData(plhs[1]));
  double* ao((double*)mxGetData(plhs[2]));

  std::copy(expectedObservations.begin(), expectedObservations.end(), eo);
  std::copy(actualObservations.begin(), actualObservations.end(), ao);
}
