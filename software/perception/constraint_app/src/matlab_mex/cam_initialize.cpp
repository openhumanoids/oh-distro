#include "mex.h"
#include "ConstraintApp.h"

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  ConstraintApp* app(new ConstraintApp());

  if ( nrhs ) {
    mexErrMsgTxt("too many parameters"); 
  }

  if ( nlhs != 1 ) {
    mexErrMsgTxt("returns exactly 1 parameter");
  }

  mwSize outdim[] = {1};
  plhs[0] = mxCreateNumericArray(1, outdim, mxUINT64_CLASS, mxREAL);

  ConstraintApp** retAddress((ConstraintApp**)mxGetData(plhs[0]));
  *retAddress = app;

  printf("created object at %p.\n", (void*)app);
}
