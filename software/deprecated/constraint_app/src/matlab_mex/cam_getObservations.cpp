#include "mex.h"
#include "ConstraintApp.h"

/**
 *  @defgroup cam_getObservations cam_getObservations
 */

/**
 * @ingroup cam_getObservations
 * @param ptr handle returned by cam_initialize()
 * @return success True or false if observations were available or unavailable.  A true return from cam_waitForObservations() will guarantee that observations are available.
 * @return observations Row vector of observations.  Individual observations are [x,y,z] tuples, and this result is a concatenation of all measured observations.
 * @return observationIds Cell array of strings associated with each tuple in the observation parameter.  This string corresponds to a link name in the OTDF.
 * @remark Matlab example: [success, observations, observationIds] = cam_getObservations(ptr)
 * @remark The C++ thread montiors the AFFORDANCE_DETECTIONS message and accumulates observations which can be retrieved with this function.
*/

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
