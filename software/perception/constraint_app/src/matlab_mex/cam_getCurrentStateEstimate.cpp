#include "mex.h"
#include "ConstraintApp.h"

/**
 *  @defgroup cam_getCurrentStateEstimate cam_getCurrentStateEstimate
 */

/**
 * @ingroup cam_getCurrentStateEstimate
 * @param ptr handle returned by cam_initialize()
 * @return the current state of the object as [x,y,z,roll,pitch,yaw] followed by the joint angles.
 * @remark Matlab example: x = cam_getCurrentStateEstimate(ptr)
 * @remark The C++ thread maintains an internal copy of the state. It can be read with this function and written with cam_setCurrentStateEstimate(ptr, x).
*/

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
