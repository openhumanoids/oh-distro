#include "mex.h"
#include "ConstraintApp.h"

/**
 *  @defgroup cam_getExpectedObservations cam_getExpectedObservations
 */

/**
 * @ingroup cam_getExpectedObservations
 * @param ptr handle returned by cam_initialize()
 * @param x matrix of NxM states where each of the N rows represents an M-element state.
 * @param observationIds Px1 cell array of strings indicating which observations are to be returned.  This will often be the result returned from cam_getObservations().
 * @return the Nx3P expected observations.  Note that each observation consist of [x,y,z] measurements (hence the 3P).
 * @remark Matlab example: z = cam_getExpectedObservations(ptr, x, observationIds)
*/

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  if ( nrhs != 3 ) {
    mexErrMsgTxt("requires exactly 3 parameters: (ptr, states, ids)"); 
  }

  if ( nlhs != 1 ) {
    mexErrMsgTxt("returns exactly 1 parameters"); 
  }

  ConstraintApp* app(*(ConstraintApp**)mxGetData(prhs[0]));

  int stateSize = app->GetStateSize();
  int stateCount = mxGetN(prhs[1]);
  if ( mxGetM(prhs[1]) != stateSize ) {
    mexErrMsgTxt("state must be SxN, where S is the size of the state");
  }

  int idSize = mxGetNumberOfElements(prhs[2]);

  std::vector<std::string> ids(idSize);
  const mxArray* idCellArray = prhs[2];
  for ( int i = 0; i < idSize; i++ ) {
    const mxArray* string_array = mxGetCell(idCellArray, i);
    int buflen = mxGetNumberOfElements(string_array) + 1;
    char* str = new char[buflen];
    mxGetString(string_array, str, buflen);
    //mexPrintf("str: [%s] %i\n", str, buflen);
    ids[i] = std::string(str);
    delete str;
  }

  int numObs = ids.size()*3;
  plhs[0] = mxCreateDoubleMatrix(numObs, stateCount, mxREAL);
  double* o((double*)mxGetData(plhs[0]));

  double* s((double*)mxGetData(prhs[1]));
  for ( int i = 0; i < stateCount; i++ ) {
    std::vector<double> state(stateSize), obs;
    std::copy(s+i*stateSize, s+i*stateSize+stateSize, state.begin());
    if ( !app->GetExpectedObservations(state, ids, obs) ) {
      mexErrMsgTxt("unable to find a link");
    }
    std::copy(obs.begin(), obs.end(), o+numObs*i);
  }
}
