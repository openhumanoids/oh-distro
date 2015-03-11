#include <mex.h>
#include "threadedController.cpp"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs < 2) mexErrMsgTxt("usage: threadedControllermex(ptr,atlas_command_channel)");

	NewQPControllerData *pdata = (NewQPControllerData*) getDrakeMexPointer(prhs[0]);

  std::shared_ptr<ThreadedControllerOptions> ctrl_opts (new ThreadedControllerOptions());
  ctrl_opts->atlas_command_channel = mxArrayToString(myGetField(prhs[1], "atlas_command_channel"));
  if (ctrl_opts->atlas_command_channel.size() == 0) {
    mexErrMsgTxt("Atlas command channel cannot be empty");
  }

	controllerLoop(pdata, ctrl_opts);
}