#include "mex.h"
#include "ConstraintApp.h"
#include <vector>

typedef std::vector<ConstraintApp*> ConstraintAppVector;
static ConstraintAppVector cam_objects;

static void cleanup(void)
{
  for ( ConstraintAppVector::iterator iter = cam_objects.begin(); 
	iter != cam_objects.end(); ++iter ) {
    mexPrintf("deleteing cam object at %p.\n", *iter);
    delete *iter;
  }
  cam_objects.clear();
}

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

  cam_objects.push_back(app);

  mexPrintf("created cam object at %p.  %i cam's exist.\n", (void*)app, (int)cam_objects.size());

  mexAtExit(cleanup);
}
