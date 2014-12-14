#include "mex.h"
#include "ConstraintApp.h"
/**
 *  @defgroup cam_waitForObservations cam_waitForObservations
 */

/**
 * @ingroup cam_waitForObservations
 * @param ptr handle returned by cam_initialize()
 * @param timeout time to wait in milliseconds
 * @return true if observations are ready; false if timeout expired
 * @remark Matlab example: success = cam_waitForObservations(ptr, 5000)
 * @remark Polling function to determine if measurements are ready
*/
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  if ( nrhs != 2 ) {
    mexErrMsgTxt("requires exactly 2 parameters"); 
  }

  if ( nlhs != 1 ) {
    mexErrMsgTxt("returns exactly 1 parameters"); 
  }

  ConstraintApp* app(*(ConstraintApp**)mxGetData(prhs[0]));

  double timeout_ms = *(double*)mxGetPr(prhs[1]);

  bool success = app->WaitForObservations((unsigned int)timeout_ms);

  plhs[0] = mxCreateLogicalScalar(success);
}
