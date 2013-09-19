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
  std::vector<std::string> observationIds;
  bool res = app->GetObservations(actualObservations, observationIds);

  plhs[0] = mxCreateLogicalScalar(res);
  plhs[1] = mxCreateDoubleMatrix(1, actualObservations.size(), mxREAL);
  mwSize dims[1] = { observationIds.size() };
  plhs[2] = mxCreateCellArray(1, dims);

  for ( int i = 0; i < observationIds.size(); i++ ) {
    mxArray* str = mxCreateString(observationIds[i].c_str());
    mxSetCell(plhs[2], i, str);
  }

  double* ao((double*)mxGetData(plhs[1]));
  std::copy(actualObservations.begin(), actualObservations.end(), ao);
}
