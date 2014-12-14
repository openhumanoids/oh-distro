#include "mex.h"
#include "ConstraintApp.h"

/**
 *  @defgroup cam_setCurrentStateEstimate cam_setCurrentStateEstimate
 */

/**
 * @ingroup cam_setCurrentStateEstimate
 * @param ptr handle returned by cam_initialize()
 * @param x new state row vector as [x,y,z,roll,pitch,yaw] followed by the joint angles.
 * @remark Matlab example: cam_setCurrentStateEstimate(ptr, s)
 * @remark The C++ thread maintains an internal copy of the state. It can be written with this function and read with cam_getCurrentStateEstimate(ptr).
*/

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
