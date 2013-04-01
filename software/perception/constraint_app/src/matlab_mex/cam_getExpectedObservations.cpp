#include "mex.h"
#include "ConstraintApp.h"

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

  int idSize = mxGetN(prhs[2]) * mxGetM(prhs[2]);

  std::vector<int> ids(idSize);
  double* is((double*)mxGetData(prhs[2]));
  std::copy(is, is+idSize, ids.begin());

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
