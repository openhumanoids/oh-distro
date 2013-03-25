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

  std::vector<double> actualObservations;
  std::vector<int> observationIds;
  bool res = app->GetObservations(actualObservations, observationIds);

  plhs[0] = mxCreateLogicalScalar(res);
  plhs[1] = mxCreateDoubleMatrix(1, actualObservations.size(), mxREAL);
  plhs[2] = mxCreateDoubleMatrix(1, observationIds.size(), mxREAL);

  double* ao((double*)mxGetData(plhs[1]));
  double* id((double*)mxGetData(plhs[2]));

  std::copy(actualObservations.begin(), actualObservations.end(), ao);
  std::copy(observationIds.begin(), observationIds.end(), id);
}
