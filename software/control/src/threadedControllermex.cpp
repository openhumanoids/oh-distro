#include <mex.h>
#include "threadedController.cpp"
#include "drake/util/drakeMexUtil.h"
#include "drake/systems/controllers/controlMexUtil.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (nrhs < 2) mexErrMsgTxt("usage: threadedControllermex(ptr,atlas_command_channel)");

	NewQPControllerData *pdata = (NewQPControllerData*) getDrakeMexPointer(prhs[0]);

  std::shared_ptr<ThreadedControllerOptions> ctrl_opts (new ThreadedControllerOptions());
  ctrl_opts->atlas_command_channel = mxArrayToString(myGetField(prhs[1], "atlas_command_channel"));
  ctrl_opts->robot_behavior_channel = mxArrayToString(myGetField(prhs[1], "robot_behavior_channel"));
  ctrl_opts->max_infocount = (int) mxGetScalar(myGetField(prhs[1], "max_infocount"));
  if (ctrl_opts->atlas_command_channel.size() == 0) {
    mexErrMsgTxt("Atlas command channel cannot be empty");
  }
  if (ctrl_opts->robot_behavior_channel.size() == 0) {
    mexErrMsgTxt("Atlas behavior channel cannot be empty");
  }

	controllerLoop(pdata, ctrl_opts);
}
