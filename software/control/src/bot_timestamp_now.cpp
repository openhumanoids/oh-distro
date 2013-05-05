#include <mex.h>
#include <bot_core/timestamp.h>
#include <cstring>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 0) {
    mexErrMsgIdAndTxt("drc:bot_timestamp_now:arguments","Too many input arguments");
  }
  if (nlhs > 1) {
    mexErrMsgIdAndTxt("drc:bot_timestamp_now:arguments","Too many output arguments");
  }

  int64_t now = bot_timestamp_now();
  plhs[0] = mxCreateNumericMatrix(1,1,mxINT64_CLASS,mxREAL);
  memcpy(mxGetData(plhs[0]),&now,sizeof(int64_t));
  return;
}
