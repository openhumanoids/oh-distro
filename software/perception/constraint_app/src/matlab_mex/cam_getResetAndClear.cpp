#include "mex.h"
#include "ConstraintApp.h"

/**
 *  @defgroup cam_getResetAndClear cam_getResetAndClear
 */

/**
 * @ingroup cam_getResetAndClear
 * @param ptr handle returned by cam_initialize()
 * @return true if an AFFORDANCE_FIT message was received which would require the filter to be reset
 * @remark Matlab example: resetOccurred = cam_getResetAndClear(ptr)
 * @remark LCM messages may require that the filter be reset.  Poll this function to determine if this is the case.  If it returns true, then cam_getCurrentStateEstimate(ptr) can be called to get the new state.
*/

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  if ( nrhs != 1 ) {
    mexErrMsgTxt("requires exactly 1 parameters"); 
  }

  ConstraintApp* app(*(ConstraintApp**)mxGetData(prhs[0]));

  if ( nlhs != 1 ) {
    mexErrMsgTxt("returns exactly 1 parameters");
  }

  bool res = app->GetResetAndClear();

  plhs[0] = mxCreateLogicalScalar(res);
}
