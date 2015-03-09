#include <mex.h>
#include "threadedController.cpp"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs < 1) mexErrMsgTxt("usage: threadedControllermex(ptr)");

	NewQPControllerData *pdata = (NewQPControllerData*) getDrakeMexPointer(prhs[0]);

	controllerLoop(pdata);
}