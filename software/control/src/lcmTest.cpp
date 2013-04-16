/* Calls the perception teams' MAP interface API */

#include <string.h>
#include <mex.h>
#include <lcm/lcm-cpp.hpp>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  lcm::LCM* plcm;

  if (nrhs<1) {
    plcm = new lcm::LCM();
    if (!plcm->good())
      mexErrMsgIdAndTxt("LCM error", "lcm not good"); 
      
    // return a pointer to the model
    mxClassID cid;
    if (sizeof(plcm)==4) cid = mxUINT32_CLASS;
    else if (sizeof(plcm)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("DRC:mapAPIWrapper:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&plcm,sizeof(plcm));

    return;
  }

  // load the data ptr
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:mapAPIWrapper:BadInputs","the first argument should be the ptr");
  memcpy(&plcm,mxGetData(prhs[0]),sizeof(plcm));

  delete plcm;
} 
